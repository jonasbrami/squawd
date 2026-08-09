"""The deep-perception sidecar (deep-perception plan §1): a registry over
models/ plus a Starlette service serving host-GPU YOLO-World / SAM 2.1
inference to the in-container DeepClient. Heavy deps (torch, ultralytics,
uvicorn) are imported lazily inside the load path and main() — importing this
package pulls only numpy, so the plain project venv can import every module.
"""
