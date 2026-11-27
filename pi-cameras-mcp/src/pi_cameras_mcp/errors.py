from mcp.server.mcpserver.exceptions import ToolError


class CameraError(ToolError):
    """A camera failure the caller can fix: missing device, busy camera, no frame."""

    def __init__(self, message: str, *, status: int = 400) -> None:
        super().__init__(message)
        self.status = status
