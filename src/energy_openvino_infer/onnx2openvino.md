sudo apt install python3.12-venv
python3 -m venv ~/ov_env
source ~/ov_env/bin/activate
pip install openvino
python3 -c "
from openvino import save_model, Core
core = Core()
model = core.read_model('/home/jlcv/infantry/src/energy_openvino_infer/models/best.onnx')
save_model(model, '/home/jlcv/infantry/src/energy_openvino_infer/models/best.xml')
"
deactivate