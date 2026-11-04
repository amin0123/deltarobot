"""Render a standalone overview of the authored scene without moving any bodies."""
from pathlib import Path
import argparse
ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--output", default=str(ROOT / "artifacts/overview.png"))
args, _ = parser.parse_known_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1280, "height": 720, "multi_gpu": False})
import omni.usd
import omni.replicator.core as rep
from PIL import Image
omni.usd.get_context().open_stage(str(ROOT / "assets/generated/sorting_cell.usdc"))
for _ in range(10):
    app.update()
product = rep.create.render_product("/World/Overview", (1280, 720))
annotator = rep.AnnotatorRegistry.get_annotator("rgb")
annotator.attach(product)
rep.orchestrator.step(rt_subframes=8, delta_time=0.0)
Image.fromarray(annotator.get_data()).save(args.output)
app.close()
