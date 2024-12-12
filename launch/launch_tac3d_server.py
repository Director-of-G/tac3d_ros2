import argparse
import yaml
import subprocess

TAC3D_SERVER_ROOT = '/home/jyp/hardware/tac3d/Tac3D-SDK-v3.2.1/Tac3D-Core/linux-x86_64'

def load_config(yaml_file):
    parameters = {
        "serials": [],
        "cam_ids": [],
        "ports": []
    }
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)['tac3d']
        for _, finger_config in config.items():
            parameters['serials'].append(finger_config['serial'])
            parameters['cam_ids'].append(finger_config['cam_id'])
            parameters['ports'].append(finger_config['port'])
    return parameters

def start_in_new_terminal(base_dir, serials, cam_ids, ports):
    terminal_command = ["gnome-terminal", "--window"]
    for i, (serial, cam_id, port) in enumerate(zip(serials, cam_ids, ports)):
        command = f"cd {base_dir} && ./Tac3D -c ./config/{serial} -d {cam_id} -i 127.0.0.1 -p {port}; exec bash"
        if i > 0:
            terminal_command.extend(["--tab", "-e", f"bash -c '{command}'"])
        else:
            terminal_command.extend(["-e", f"bash -c '{command}'"])

    subprocess.Popen(terminal_command)
    print("Commands launched in a new terminal window with tabs.")

def main():
    parser = argparse.ArgumentParser(description="Launch Tac3D commands.")
    parser.add_argument("--yaml-config", type=str, required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--tac3d-server-root", type=str, required=True, default=TAC3D_SERVER_ROOT, help="Root directory of the Tac3D server.")
    args = parser.parse_args()

    params = load_config(args.yaml_config)
    serials = params['serials']
    cam_ids = params['cam_ids']
    ports = params['ports']

    start_in_new_terminal(args.tac3d_server_root, serials, cam_ids, ports)

if __name__ == "__main__":
    main()