import libtmux
import subprocess
import os

class ProcessManager:
    def __init__(self, session_name="franka_ui_session"):
        self.server = libtmux.Server()
        self.session_name = session_name
        
        # Connect to existing session or create new
        try:
            self.session = self.server.sessions.get(session_name=self.session_name)
        except Exception:
            self.session = self.server.new_session(session_name=self.session_name, detached=True)

    def start_process(self, window_name, commands):
        # Kill if exists
        try:
            window = self.session.windows.get(window_name=window_name)
            window.kill_window()
        except Exception:
            pass
            
        if isinstance(commands, str):
            commands = [commands]
            
        window = self.session.new_window(window_name=window_name, attach=False)
        
        for i, cmd in enumerate(commands):
            if i == 0:
                pane = window.attached_pane
            else:
                pane = window.split_window(attach=False)
            pane.send_keys(cmd)
            
        return window

    def stop_process(self, window_name):
        try:
            window = self.session.windows.get(window_name=window_name)
            window.kill_window()
        except Exception:
            pass

    def is_running(self, window_name):
        try:
            window = self.session.windows.get(window_name=window_name)
            for pane in window.panes:
                if pane.pane_dead == '1':
                    return False
            return True
        except Exception:
            return False

    def open_terminal(self, window_name):
        # Attach to the specific tmux window via gnome-terminal. 
        # Use env -i to strip pixi/conda environment vars causing glib/atk conflicts.
        # Wrap the target in quotes to handle spaces in window_name.
        # Use while true loop to keep terminal open and auto-reconnect on restart.
        cmd = f"env -i DISPLAY=$DISPLAY HOME=$HOME USER=$USER PATH=/usr/local/bin:/usr/bin:/bin gnome-terminal -- bash -c \"while true; do tmux attach-session -t '{self.session_name}:{window_name}'; sleep 1; done\""
        subprocess.Popen(cmd, shell=True)
