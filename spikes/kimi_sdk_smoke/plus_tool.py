"""M0 fallback-harness tool module for the agent.yaml smoke (loaded by dotted path)."""
from kosong.tooling import CallableTool2, ToolOk, ToolReturnValue
from pydantic import BaseModel


class PlusParams(BaseModel):
    a: float
    b: float


class Plus(CallableTool2[PlusParams]):
    name: str = "plus"
    description: str = "Add two numbers and return the sum."
    params: type[PlusParams] = PlusParams

    async def __call__(self, params: PlusParams) -> ToolReturnValue:
        return ToolOk(output=str(params.a + params.b))
