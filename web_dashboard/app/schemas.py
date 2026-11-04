"""API 响应模型：字段与数据库、ROS 契约保持一致。"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

ControlCommand = Literal["start", "pause", "resume", "skip", "stop", "manual"]

# 轮次 / 事件 ID：字母数字开头，8-64 位，允许 - 与 _
ID_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_-]{7,63}$"


class PoiResultOut(BaseModel):
    poi: str
    sequence: int
    status_code: int | None
    status_name: str | None
    skipped: bool
    simulated: bool
    detail: str | None
    detections: list[dict] | None = Field(
        default=None, description="解析后的 detections_json；解析失败为 null"
    )
    checked_at: str | None
    duration_s: float | None


class RoundSummary(BaseModel):
    round_id: str
    mode: str
    source: str
    is_demo: bool
    status: str
    outcome: str | None
    operator_id: str | None
    ros_state: str | None
    summary: str | None
    error_message: str | None
    total_pois: int | None = Field(
        default=None,
        description="计划点位总数；为 null 时前端不显示进度百分比（不臆造）",
    )
    started_at: str
    finished_at: str | None
    recorded_count: int = Field(description="已记录的巡检点数量（不含跳过）")
    processed_count: int = Field(description="已处理的点位数量（含跳过）")
    abnormal_count: int = Field(description="判定为 ABNORMAL 的点位数量")


class RoundDetail(RoundSummary):
    results: list[PoiResultOut]


class RoundListResponse(BaseModel):
    items: list[RoundSummary]
    total: int
    limit: int
    offset: int


class RobotStatusOut(BaseModel):
    online: bool
    adapter: str
    state_text: str | None
    detail: str
    checked_at: str


class StatusResponse(BaseModel):
    server_time: str
    robot: RobotStatusOut
    current_round: RoundDetail | None


class HealthResponse(BaseModel):
    status: Literal["ok", "degraded"]
    database: Literal["ok", "error"]
    robot: RobotStatusOut
    server_time: str


class ControlRequest(BaseModel):
    cmd: ControlCommand
    arg: str = Field(default="", max_length=64)
    operator_id: str = Field(default="", max_length=64)


class ControlResponse(BaseModel):
    success: bool
    message: str
    current_state: str | None = None
    dispatched: bool = Field(
        default=False,
        description="true 表示 Bridge 已确认收到 ROS 控制服务响应；离线时恒为 false",
    )


# ---------------------------------------------------------------------------
# ROS Bridge → 后端 数据接入（仅 /api/ingest/*，需要 X-Bridge-Token）
# ---------------------------------------------------------------------------


class IngestRoundStart(BaseModel):
    round_id: str = Field(pattern=ID_PATTERN)
    mode: Literal["inspection", "manual"] = "inspection"
    operator_id: str | None = Field(default=None, max_length=64)
    started_at: str | None = Field(default=None, max_length=64)
    ros_state: str | None = Field(default=None, max_length=256)
    total_pois: int | None = Field(
        default=None,
        ge=0,
        le=1000,
        description="本轮计划点位总数（可选）；不提供则前端不显示进度百分比",
    )


class IngestState(BaseModel):
    status: Literal["running", "paused"]
    ros_state: str | None = Field(default=None, max_length=256)


class IngestResult(BaseModel):
    event_id: str = Field(pattern=ID_PATTERN)
    poi: str = Field(min_length=1, max_length=64)
    status_code: int = Field(ge=0, le=6)
    simulated: bool = False
    detail: str | None = Field(default=None, max_length=2000)
    detections_json: str | None = Field(default=None, max_length=20000)
    checked_at: str | None = Field(default=None, max_length=64)
    duration_s: float | None = Field(default=None, ge=0)


class IngestFinish(BaseModel):
    status: Literal["completed", "stopped", "aborted"]
    summary: str | None = Field(default=None, max_length=500)
    error_message: str | None = Field(default=None, max_length=2000)
    finished_at: str | None = Field(default=None, max_length=64)


# ---------------------------------------------------------------------------
# 手动点名检查（与自动巡检完全独立，不进 inspection_round / poi_result）
# ---------------------------------------------------------------------------


class ManualInspectionOut(BaseModel):
    manual_id: str
    poi: str
    operator_id: str | None
    status: str = Field(description="执行状态：running / completed / stopped / aborted")
    ros_state: str | None
    status_code: int | None = Field(
        description="正式视觉检查状态（0-6）；null 表示没有正式结果，不是正常"
    )
    status_name: str | None
    simulated: bool
    detail: str | None
    detections: list[dict] | None = Field(
        default=None, description="解析后的 detections_json；解析失败为 null"
    )
    started_at: str
    checked_at: str | None
    finished_at: str | None
    error_message: str | None


class ManualListResponse(BaseModel):
    items: list[ManualInspectionOut]
    total: int
    limit: int
    offset: int


class IngestManualStart(BaseModel):
    manual_id: str = Field(pattern=ID_PATTERN)
    poi: str = Field(min_length=1, max_length=64)
    operator_id: str | None = Field(default=None, max_length=64)
    started_at: str | None = Field(default=None, max_length=64)
    ros_state: str | None = Field(default=None, max_length=256)


class IngestManualState(BaseModel):
    ros_state: str | None = Field(default=None, max_length=256)


class IngestManualResult(BaseModel):
    event_id: str = Field(pattern=ID_PATTERN)
    status_code: int = Field(ge=0, le=6)
    simulated: bool = False
    detail: str | None = Field(default=None, max_length=2000)
    detections_json: str | None = Field(default=None, max_length=20000)
    checked_at: str | None = Field(default=None, max_length=64)


class IngestManualFinish(BaseModel):
    status: Literal["completed", "stopped", "aborted"]
    error_message: str | None = Field(default=None, max_length=2000)
    finished_at: str | None = Field(default=None, max_length=64)


# ---------------------------------------------------------------------------
# 统一异常事件（网页侧建议契约，待机器人端确认）
# ---------------------------------------------------------------------------


class EventOut(BaseModel):
    event_id: str
    timestamp: str = Field(description="事件发生时间（ISO 8601，UTC）")
    event_type: str
    source: str
    severity: str
    message: str
    details: dict | None = None
    round_id: str | None = None
    poi: str | None = None
    simulated: bool = Field(
        default=False, description="true 表示模拟事件，不得当作真实机器人事件"
    )
    status: str = Field(description="处理状态：open / acknowledged / resolved")
    handled_by: str | None = None
    handled_at: str | None = None


class EventListResponse(BaseModel):
    items: list[EventOut]
    total: int
    limit: int
    offset: int


class IngestEvent(BaseModel):
    """机器人端 / 开发工具上报事件。字段为建议契约，待机器人端确认。"""

    event_id: str = Field(pattern=ID_PATTERN)
    event_type: Literal[
        "alarm_detected",
        "vision_abnormal",
        "voice_broadcast",
        "comm_failure",
        "comm_restored",
    ]
    timestamp: str | None = Field(default=None, max_length=64)
    source: str = Field(min_length=1, max_length=32)
    severity: Literal["info", "warning", "critical"] = "warning"
    message: str = Field(min_length=1, max_length=500)
    details: dict | None = None
    round_id: str | None = Field(default=None, pattern=ID_PATTERN)
    poi: str | None = Field(default=None, max_length=64)
    simulated: bool = False


class EventAckRequest(BaseModel):
    operator_id: str = Field(default="", max_length=64)
    status: Literal["acknowledged", "resolved"] = "acknowledged"
