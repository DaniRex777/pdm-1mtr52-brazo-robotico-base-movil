"""
db.py — Módulo de acceso a PostgreSQL (async con asyncpg)
Lab Procesos Industriales — PUCP

ARCHIVO NUEVO — no existía en el sistema original.

Responsabilidades:
  - Crear y gestionar el pool de conexiones
  - Insertar lecturas de sensores y eventos
  - Consultar historial con filtros por máquina, nodo, variable y rango de fechas
  - Inicializar las tablas y datos maestros al arrancar

Uso desde server.py:
  from db import Database
  db = Database()
  await db.connect()
  await db.insert_reading(node_slug="nodo_1", machine_slug="evaporador",
                          variable_slug="temperatura", value=84.3)
  await db.disconnect()

Instalación de asyncpg:
  pip install asyncpg
"""

import asyncio
import csv
import io
import logging
from datetime import datetime, timezone
from typing import List, Optional

logger = logging.getLogger("db")

try:
    import asyncpg
    ASYNCPG_AVAILABLE = True
except ImportError:
    ASYNCPG_AVAILABLE = False
    logger.warning(
        "asyncpg no encontrado. La base de datos PostgreSQL no estará disponible.\n"
        "Instalar con: pip install asyncpg"
    )


# ─────────────────────────────────────────────────────────────
# DDL — se ejecuta al arrancar si las tablas no existen
# ─────────────────────────────────────────────────────────────

DDL = """
CREATE TABLE IF NOT EXISTS machines (
    id          SERIAL PRIMARY KEY,
    slug        VARCHAR(64) UNIQUE NOT NULL,
    name        VARCHAR(128) NOT NULL,
    description TEXT,
    active      BOOLEAN DEFAULT TRUE,
    created_at  TIMESTAMPTZ DEFAULT NOW()
);

CREATE TABLE IF NOT EXISTS sensor_nodes (
    id          SERIAL PRIMARY KEY,
    machine_id  INTEGER REFERENCES machines(id),
    slug        VARCHAR(64) UNIQUE NOT NULL,
    name        VARCHAR(128),
    mqtt_prefix VARCHAR(256),
    active      BOOLEAN DEFAULT TRUE,
    created_at  TIMESTAMPTZ DEFAULT NOW()
);

CREATE TABLE IF NOT EXISTS variable_types (
    id              SERIAL PRIMARY KEY,
    slug            VARCHAR(64) UNIQUE NOT NULL,
    name            VARCHAR(128),
    unit            VARCHAR(32),
    warn_threshold  FLOAT,
    max_threshold   FLOAT
);

CREATE TABLE IF NOT EXISTS sensor_readings (
    id          BIGSERIAL PRIMARY KEY,
    node_id     INTEGER REFERENCES sensor_nodes(id),
    variable_id INTEGER REFERENCES variable_types(id),
    value       FLOAT NOT NULL,
    recorded_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_readings_node_time
    ON sensor_readings(node_id, recorded_at DESC);

CREATE INDEX IF NOT EXISTS idx_readings_var_time
    ON sensor_readings(variable_id, recorded_at DESC);

CREATE TABLE IF NOT EXISTS system_events (
    id          BIGSERIAL PRIMARY KEY,
    event_type  VARCHAR(64) NOT NULL,
    detail      TEXT,
    recorded_at TIMESTAMPTZ DEFAULT NOW()
);
"""


class Database:
    """
    Clase principal para acceso a PostgreSQL.
    Usa un pool de conexiones asyncpg para operaciones concurrentes.
    """

    def __init__(self, host: str, port: int, database: str,
                 user: str, password: str, min_size: int = 2, max_size: int = 10):
        """
        Construye el DSN de PostgreSQL. La conexión real ocurre en connect().

        Args:
            host, port, database, user, password: credenciales (ver config.py).
            min_size, max_size (int): tamaño del pool de conexiones asyncpg.
        """
        self.dsn = f"postgresql://{user}:{password}@{host}:{port}/{database}"
        self.min_size = min_size
        self.max_size = max_size
        self._pool = None
        self.available = ASYNCPG_AVAILABLE

        # Caché en memoria de IDs para evitar queries repetidas
        # { "machine_slug": id, ... }
        self._machine_ids: dict = {}
        # { "node_slug": id, ... }
        self._node_ids: dict = {}
        # { "variable_slug": id, ... }
        self._variable_ids: dict = {}

    # ─────────────────────────────────────────
    # Conexión y desconexión
    # ─────────────────────────────────────────

    async def connect(self):
        """Crea el pool de conexiones y ejecuta el DDL si es necesario."""
        if not self.available:
            logger.warning("asyncpg no disponible, DB desactivada.")
            return

        try:
            logger.info("Conectando a PostgreSQL...")
            self._pool = await asyncpg.create_pool(
                dsn=self.dsn,
                min_size=self.min_size,
                max_size=self.max_size,
                command_timeout=10
            )
            await self._init_schema()
            logger.info("PostgreSQL conectado y esquema verificado.")
        except Exception as e:
            logger.error(f"Error conectando a PostgreSQL: {e}")
            self._pool = None

    async def disconnect(self):
        """Cierra el pool de conexiones."""
        if self._pool:
            await self._pool.close()
            logger.info("PostgreSQL desconectado.")

    @property
    def connected(self) -> bool:
        """
        Returns:
            bool: True si el pool de conexiones está activo.
        """
        return self._pool is not None

    # ─────────────────────────────────────────
    # Inicialización de esquema y datos maestros
    # ─────────────────────────────────────────

    async def _init_schema(self):
        """Crea las tablas si no existen."""
        async with self._pool.acquire() as conn:
            await conn.execute(DDL)

    async def upsert_masters(self, sensor_nodes_config: list):
        """
        Inserta o actualiza las tablas maestras (machines, sensor_nodes, variable_types)
        a partir de la lista SENSOR_NODES de config.py.

        Se llama una vez al arrancar el servidor.
        """
        if not self.connected:
            return

        async with self._pool.acquire() as conn:
            for node_cfg in sensor_nodes_config:
                # Upsert de machine
                machine_id = await conn.fetchval("""
                    INSERT INTO machines (slug, name)
                    VALUES ($1, $2)
                    ON CONFLICT (slug) DO UPDATE SET name = EXCLUDED.name
                    RETURNING id
                """, node_cfg["machine_slug"], node_cfg["machine_name"])

                # Upsert de sensor_node
                node_id = await conn.fetchval("""
                    INSERT INTO sensor_nodes (machine_id, slug, mqtt_prefix)
                    VALUES ($1, $2, $3)
                    ON CONFLICT (slug) DO UPDATE
                        SET machine_id = EXCLUDED.machine_id,
                            mqtt_prefix = EXCLUDED.mqtt_prefix
                    RETURNING id
                """, machine_id, node_cfg["node_slug"], node_cfg["mqtt_prefix"])

                # Upsert de variable_types
                for var in node_cfg["variables"]:
                    var_id = await conn.fetchval("""
                        INSERT INTO variable_types (slug, unit, warn_threshold, max_threshold)
                        VALUES ($1, $2, $3, $4)
                        ON CONFLICT (slug) DO UPDATE
                            SET unit = EXCLUDED.unit,
                                warn_threshold = EXCLUDED.warn_threshold,
                                max_threshold  = EXCLUDED.max_threshold
                        RETURNING id
                    """, var["slug"], var["unit"], var["warn"], var["max"])

                    # Poblar caché
                    self._variable_ids[var["slug"]] = var_id

                self._machine_ids[node_cfg["machine_slug"]] = machine_id
                self._node_ids[node_cfg["node_slug"]] = node_id

        logger.info(f"Datos maestros sincronizados: {len(sensor_nodes_config)} nodo(s).")

    # ─────────────────────────────────────────
    # Escritura
    # ─────────────────────────────────────────

    async def insert_reading(self, node_slug: str, variable_slug: str, value: float):
        """
        Inserta una lectura de sensor.

        Parámetros:
          node_slug     → slug del nodo (ej: "nodo_1")
          variable_slug → slug de la variable (ej: "temperatura")
          value         → valor numérico medido
        """
        if not self.connected:
            return

        node_id = self._node_ids.get(node_slug)
        variable_id = self._variable_ids.get(variable_slug)

        if node_id is None or variable_id is None:
            logger.warning(
                f"insert_reading: nodo '{node_slug}' o variable '{variable_slug}' "
                f"no encontrados en caché. ¿Se ejecutó upsert_masters()?"
            )
            return

        try:
            async with self._pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO sensor_readings (node_id, variable_id, value)
                    VALUES ($1, $2, $3)
                """, node_id, variable_id, float(value))
        except Exception as e:
            logger.error(f"Error insertando lectura: {e}")

    async def insert_reading_batch(self, readings: list):
        """
        Inserta múltiples lecturas en una sola transacción.

        Parámetro:
          readings → lista de dicts con claves: node_slug, variable_slug, value
        """
        if not self.connected or not readings:
            return

        rows = []
        for r in readings:
            node_id = self._node_ids.get(r["node_slug"])
            var_id  = self._variable_ids.get(r["variable_slug"])
            if node_id and var_id:
                rows.append((node_id, var_id, float(r["value"])))

        if not rows:
            return

        try:
            async with self._pool.acquire() as conn:
                await conn.executemany("""
                    INSERT INTO sensor_readings (node_id, variable_id, value)
                    VALUES ($1, $2, $3)
                """, rows)
        except Exception as e:
            logger.error(f"Error en insert_reading_batch: {e}")

    async def insert_event(self, event_type: str, detail: str = ""):
        """Inserta un evento del sistema."""
        if not self.connected:
            return
        try:
            async with self._pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO system_events (event_type, detail)
                    VALUES ($1, $2)
                """, event_type, detail)
        except Exception as e:
            logger.error(f"Error insertando evento: {e}")

    # ─────────────────────────────────────────
    # Lectura — historial de sensores
    # ─────────────────────────────────────────

    async def query_readings(
        self,
        machine_slug:  Optional[str] = None,
        node_slug:     Optional[str] = None,
        variable_slug: Optional[str] = None,
        since:         Optional[datetime] = None,
        until:         Optional[datetime] = None,
        limit:         int = 500,
        agg:           Optional[str] = None,   # None | 'hour' | 'day'
    ) -> List[dict]:
        """
        Consulta lecturas históricas con filtros opcionales.

        Parámetros:
          machine_slug  → filtrar por máquina
          node_slug     → filtrar por nodo sensor
          variable_slug → filtrar por tipo de variable
          since / until → rango de fechas (datetime con timezone)
          limit         → máximo de registros a retornar
          agg           → agregación temporal: None=sin agregar, 'hour'=por hora, 'day'=por día

        Retorna lista de dicts con: recorded_at, value, node_slug, machine_slug, variable_slug, unit
        """
        if not self.connected:
            return []

        # Construir query dinámico
        conditions = []
        params = []
        p = 1  # índice de parámetro $N

        if machine_slug:
            conditions.append(f"m.slug = ${p}")
            params.append(machine_slug)
            p += 1

        if node_slug:
            conditions.append(f"n.slug = ${p}")
            params.append(node_slug)
            p += 1

        if variable_slug:
            conditions.append(f"v.slug = ${p}")
            params.append(variable_slug)
            p += 1

        if since:
            conditions.append(f"r.recorded_at >= ${p}")
            params.append(since)
            p += 1

        if until:
            conditions.append(f"r.recorded_at <= ${p}")
            params.append(until)
            p += 1

        where_clause = ("WHERE " + " AND ".join(conditions)) if conditions else ""

        # Agregaciones soportadas:
        #   'day'    → date_trunc por día
        #   'hour'   → date_trunc por hora
        #   'minute' → date_trunc por minuto
        #   '30s'    → trunc a bloques de 30 segundos (epoch / 30 * 30)
        if agg == "day":
            time_trunc = "date_trunc('day', r.recorded_at)"
            group_expr = time_trunc
        elif agg == "hour":
            time_trunc = "date_trunc('hour', r.recorded_at)"
            group_expr = time_trunc
        elif agg == "minute":
            time_trunc = "date_trunc('minute', r.recorded_at)"
            group_expr = time_trunc
        elif agg == "30s":
            # Redondea al múltiplo de 30s más cercano hacia abajo
            time_trunc = "to_timestamp(floor(extract(epoch from r.recorded_at)/30)*30)"
            group_expr = "floor(extract(epoch from r.recorded_at)/30)"
        else:
            time_trunc = None
            group_expr = None

        if time_trunc:
            # Query con agregación: promedio por intervalo de tiempo
            sql = f"""
                SELECT
                    {time_trunc}                     AS recorded_at,
                    AVG(r.value)                     AS value,
                    MIN(r.value)                     AS value_min,
                    MAX(r.value)                     AS value_max,
                    n.slug                           AS node_slug,
                    m.slug                           AS machine_slug,
                    v.slug                           AS variable_slug,
                    v.unit                           AS unit
                FROM sensor_readings r
                JOIN sensor_nodes    n ON n.id = r.node_id
                JOIN machines        m ON m.id = n.machine_id
                JOIN variable_types  v ON v.id = r.variable_id
                {where_clause}
                GROUP BY {group_expr}, n.slug, m.slug, v.slug, v.unit
                ORDER BY recorded_at DESC
                LIMIT ${p}
            """
        else:
            sql = f"""
                SELECT
                    r.recorded_at,
                    r.value,
                    n.slug  AS node_slug,
                    m.slug  AS machine_slug,
                    v.slug  AS variable_slug,
                    v.unit  AS unit
                FROM sensor_readings r
                JOIN sensor_nodes    n ON n.id = r.node_id
                JOIN machines        m ON m.id = n.machine_id
                JOIN variable_types  v ON v.id = r.variable_id
                {where_clause}
                ORDER BY r.recorded_at DESC
                LIMIT ${p}
            """

        params.append(limit)

        try:
            async with self._pool.acquire() as conn:
                rows = await conn.fetch(sql, *params)
                return [dict(r) for r in rows]
        except Exception as e:
            logger.error(f"Error en query_readings: {e}")
            return []

    async def query_events(self, limit: int = 50) -> List[dict]:
        """Retorna los últimos N eventos del sistema."""
        if not self.connected:
            return []
        try:
            async with self._pool.acquire() as conn:
                rows = await conn.fetch("""
                    SELECT id, event_type, detail, recorded_at
                    FROM system_events
                    ORDER BY recorded_at DESC
                    LIMIT $1
                """, limit)
                return [dict(r) for r in rows]
        except Exception as e:
            logger.error(f"Error en query_events: {e}")
            return []

    async def query_machines(self) -> List[dict]:
        """Retorna la lista de máquinas activas."""
        if not self.connected:
            return []
        try:
            async with self._pool.acquire() as conn:
                rows = await conn.fetch("""
                    SELECT id, slug, name FROM machines WHERE active = TRUE ORDER BY name
                """)
                return [dict(r) for r in rows]
        except Exception as e:
            logger.error(f"Error en query_machines: {e}")
            return []

    async def query_nodes(self, machine_slug: Optional[str] = None) -> List[dict]:
        """Retorna la lista de nodos sensores, opcionalmente filtrada por máquina."""
        if not self.connected:
            return []
        try:
            async with self._pool.acquire() as conn:
                if machine_slug:
                    rows = await conn.fetch("""
                        SELECT n.id, n.slug, n.mqtt_prefix, m.slug AS machine_slug, m.name AS machine_name
                        FROM sensor_nodes n
                        JOIN machines m ON m.id = n.machine_id
                        WHERE n.active = TRUE AND m.slug = $1
                        ORDER BY n.slug
                    """, machine_slug)
                else:
                    rows = await conn.fetch("""
                        SELECT n.id, n.slug, n.mqtt_prefix, m.slug AS machine_slug, m.name AS machine_name
                        FROM sensor_nodes n
                        JOIN machines m ON m.id = n.machine_id
                        WHERE n.active = TRUE
                        ORDER BY m.slug, n.slug
                    """)
                return [dict(r) for r in rows]
        except Exception as e:
            logger.error(f"Error en query_nodes: {e}")
            return []

    # ─────────────────────────────────────────
    # Exportación CSV
    # ─────────────────────────────────────────

    async def export_readings_csv(self, **kwargs) -> str:
        """
        Exporta lecturas a formato CSV como string.
        Acepta los mismos parámetros que query_readings().
        """
        rows = await self.query_readings(**kwargs)

        output = io.StringIO()
        writer = csv.DictWriter(
            output,
            fieldnames=["recorded_at", "machine_slug", "node_slug", "variable_slug", "unit", "value"],
            extrasaction="ignore"
        )
        writer.writeheader()
        for row in rows:
            # Convertir datetime a string ISO si es necesario
            if isinstance(row.get("recorded_at"), datetime):
                row["recorded_at"] = row["recorded_at"].isoformat()
            writer.writerow(row)

        return output.getvalue()

    # ─────────────────────────────────────────
    # Estadísticas rápidas
    # ─────────────────────────────────────────

    async def query_stats(
        self,
        machine_slug:  Optional[str] = None,
        variable_slug: Optional[str] = None,
        since:         Optional[datetime] = None,
        until:         Optional[datetime] = None,
    ) -> dict:
        """
        Retorna estadísticas (min, max, avg, count) para un rango de tiempo.
        """
        if not self.connected:
            return {}

        conditions = []
        params = []
        p = 1

        if machine_slug:
            conditions.append(f"m.slug = ${p}"); params.append(machine_slug); p += 1
        if variable_slug:
            conditions.append(f"v.slug = ${p}"); params.append(variable_slug); p += 1
        if since:
            conditions.append(f"r.recorded_at >= ${p}"); params.append(since); p += 1
        if until:
            conditions.append(f"r.recorded_at <= ${p}"); params.append(until); p += 1

        where_clause = ("WHERE " + " AND ".join(conditions)) if conditions else ""

        sql = f"""
            SELECT
                v.slug  AS variable_slug,
                v.unit,
                MIN(r.value)   AS value_min,
                MAX(r.value)   AS value_max,
                AVG(r.value)   AS value_avg,
                COUNT(r.id)    AS count
            FROM sensor_readings r
            JOIN sensor_nodes    n ON n.id = r.node_id
            JOIN machines        m ON m.id = n.machine_id
            JOIN variable_types  v ON v.id = r.variable_id
            {where_clause}
            GROUP BY v.slug, v.unit
        """

        try:
            async with self._pool.acquire() as conn:
                rows = await conn.fetch(sql, *params)
                return {r["variable_slug"]: dict(r) for r in rows}
        except Exception as e:
            logger.error(f"Error en query_stats: {e}")
            return {}

    # ─────────────────────────────────────────
    # Análisis avanzado
    # ─────────────────────────────────────────

    async def query_analytics(
        self,
        machine_slug: str,
        since: Optional[datetime] = None,
        until: Optional[datetime] = None,
    ) -> dict:
        """
        Retorna un paquete de análisis avanzado para una máquina:
          - Correlación Pearson entre T1..T4 y flujo
          - Porcentaje de tiempo sobre umbral de alerta por variable
          - Tendencia (slope) de cada variable: subiendo, bajando, estable
          - Spread térmico (máx-mín entre T1..T4 en cada timestamp)

        Todos los cálculos se hacen en PostgreSQL puro para eficiencia.
        """
        if not self.connected:
            return {}

        dt_filter = ""
        params_base: list = [machine_slug]
        p = 2

        if since:
            dt_filter += f" AND r.recorded_at >= ${p}"; params_base.append(since); p += 1
        if until:
            dt_filter += f" AND r.recorded_at <= ${p}"; params_base.append(until); p += 1

        results = {}

        async with self._pool.acquire() as conn:

            # ── 1. Estadísticas por variable con porcentaje sobre umbral ──
            warn_rows = await conn.fetch(f"""
                SELECT
                    v.slug,
                    v.unit,
                    v.warn_threshold,
                    v.max_threshold,
                    COUNT(r.id)                                      AS total,
                    MIN(r.value)                                     AS val_min,
                    MAX(r.value)                                     AS val_max,
                    AVG(r.value)                                     AS val_avg,
                    STDDEV(r.value)                                  AS val_std,
                    COUNT(CASE WHEN v.warn_threshold IS NOT NULL
                               AND r.value > v.warn_threshold
                               THEN 1 END)                           AS over_warn
                FROM sensor_readings r
                JOIN sensor_nodes   n ON n.id = r.node_id
                JOIN machines       m ON m.id = n.machine_id
                JOIN variable_types v ON v.id = r.variable_id
                WHERE m.slug = $1 {dt_filter}
                GROUP BY v.slug, v.unit, v.warn_threshold, v.max_threshold
                ORDER BY v.slug
            """, *params_base)

            for row in warn_rows:
                slug = row["slug"]
                total = row["total"] or 1
                pct_warn = round(100.0 * (row["over_warn"] or 0) / total, 1)
                results[slug] = {
                    "unit":       row["unit"],
                    "total":      total,
                    "val_min":    round(float(row["val_min"] or 0), 2),
                    "val_max":    round(float(row["val_max"] or 0), 2),
                    "val_avg":    round(float(row["val_avg"] or 0), 2),
                    "val_std":    round(float(row["val_std"] or 0), 3),
                    "warn_threshold": float(row["warn_threshold"] or 0),
                    "pct_over_warn":  pct_warn,
                }

            # ── 2. Tendencia lineal (regresión simple en SQL) ──
            # Usando la fórmula de Pearson: slope = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
            # x = segundos desde el primer registro, y = value
            trend_rows = await conn.fetch(f"""
                WITH base AS (
                    SELECT
                        v.slug,
                        EXTRACT(EPOCH FROM r.recorded_at) AS t,
                        r.value AS y
                    FROM sensor_readings r
                    JOIN sensor_nodes   n ON n.id = r.node_id
                    JOIN machines       m ON m.id = n.machine_id
                    JOIN variable_types v ON v.id = r.variable_id
                    WHERE m.slug = $1 {dt_filter}
                ),
                stats AS (
                    SELECT
                        slug,
                        COUNT(*)          AS n,
                        SUM(t)            AS sum_t,
                        SUM(y)            AS sum_y,
                        SUM(t*y)          AS sum_ty,
                        SUM(t*t)          AS sum_tt,
                        MIN(t)            AS t_min,
                        MAX(t)            AS t_max
                    FROM base
                    GROUP BY slug
                )
                SELECT
                    slug,
                    CASE
                        WHEN (n * sum_tt - sum_t * sum_t) = 0 THEN 0
                        ELSE (n * sum_ty - sum_t * sum_y) / (n * sum_tt - sum_t * sum_t)
                    END AS slope
                FROM stats
            """, *params_base)

            for row in trend_rows:
                slug = row["slug"]
                slope = float(row["slope"] or 0)
                # slope en unidades/segundo → convertir a unidades/minuto
                slope_per_min = round(slope * 60, 4)
                trend = "estable"
                if slope_per_min > 0.05:
                    trend = "subiendo"
                elif slope_per_min < -0.05:
                    trend = "bajando"
                if slug in results:
                    results[slug]["slope_per_min"] = slope_per_min
                    results[slug]["trend"] = trend

            # ── 3. Spread térmico: máx-mín entre T1..T4 por timestamp ──
            # Solo aplica si hay al menos 2 sensores de temperatura
            spread_row = await conn.fetchrow(f"""
                WITH temps AS (
                    SELECT
                        r.recorded_at,
                        r.value
                    FROM sensor_readings r
                    JOIN sensor_nodes   n ON n.id = r.node_id
                    JOIN machines       m ON m.id = n.machine_id
                    JOIN variable_types v ON v.id = r.variable_id
                    WHERE m.slug = $1
                      AND v.unit = '°C'
                      {dt_filter.replace('AND r.recorded_at', 'AND r.recorded_at')}
                ),
                spread_per_ts AS (
                    SELECT
                        recorded_at,
                        MAX(value) - MIN(value) AS spread
                    FROM temps
                    GROUP BY recorded_at
                    HAVING COUNT(*) > 1
                )
                SELECT
                    AVG(spread) AS avg_spread,
                    MAX(spread) AS max_spread,
                    MIN(spread) AS min_spread
                FROM spread_per_ts
            """, *params_base)

            if spread_row and spread_row["avg_spread"] is not None:
                results["__spread_termico__"] = {
                    "avg": round(float(spread_row["avg_spread"]), 2),
                    "max": round(float(spread_row["max_spread"]), 2),
                    "min": round(float(spread_row["min_spread"]), 2),
                    "descripcion": "Diferencia máx-mín entre sensores de temperatura en el mismo instante",
                }

            # ── 4. Correlación Pearson entre T1 y flujo ──
            corr_row = await conn.fetchrow(f"""
                WITH t1 AS (
                    SELECT r.recorded_at, r.value AS vt
                    FROM sensor_readings r
                    JOIN sensor_nodes   n ON n.id = r.node_id
                    JOIN machines       m ON m.id = n.machine_id
                    JOIN variable_types v ON v.id = r.variable_id
                    WHERE m.slug = $1 AND v.slug LIKE '%_t1' {dt_filter}
                ),
                fl AS (
                    SELECT r.recorded_at, r.value AS vf
                    FROM sensor_readings r
                    JOIN sensor_nodes   n ON n.id = r.node_id
                    JOIN machines       m ON m.id = n.machine_id
                    JOIN variable_types v ON v.id = r.variable_id
                    WHERE m.slug = $1 AND v.slug LIKE '%flujo' {dt_filter}
                )
                SELECT CORR(vt, vf) AS pearson
                FROM t1 JOIN fl USING (recorded_at)
            """, *params_base)

            if corr_row and corr_row["pearson"] is not None:
                pearson = round(float(corr_row["pearson"]), 3)
                results["__correlacion_t1_flujo__"] = {
                    "pearson": pearson,
                    "interpretacion": (
                        "correlación fuerte positiva" if pearson > 0.7 else
                        "correlación fuerte negativa" if pearson < -0.7 else
                        "correlación moderada" if abs(pearson) > 0.4 else
                        "correlación débil"
                    )
                }

        return results
