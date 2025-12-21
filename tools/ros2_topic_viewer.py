#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_yaml


# ==================================================
# App Info
# ==================================================
APP_NAME = "Jarabot ROS2 Topic Viewer"
APP_VERSION = "v2.0.0"
APP_AUTHOR = "Yonguk Cho"


# ==================================================
# Helpers
# ==================================================
def run_cmd_capture(cmd, timeout_sec=None):
    """
    Run command and return (ok:bool, output:str).
    - Shows stderr too.
    - If timeout, returns partial output with note.
    """
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_sec
        )
        ok = (p.returncode == 0)
        return ok, p.stdout
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "")
        out += f"\n\n[NOTE] Timed out after {timeout_sec}s (command runs continuously). " \
               f"Increase 'Duration' or stop it.\n"
        return True, out
    except Exception as e:
        return False, str(e)


def run_cmd(cmd, timeout=None):
    """
    cmd: list[str]
    returns: (returncode, stdout, stderr)
    """
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        out, err = p.communicate(timeout=timeout)
        return p.returncode, out, err
    except subprocess.TimeoutExpired:
        p.kill()
        out, err = p.communicate()
        return 124, out, (err + "\n[timeout]\n")
    except Exception as e:
        return 1, "", f"{type(e).__name__}: {e}\n"


# ==================================================
# ROS Node
# ==================================================
class TopicViewerNode(Node):
    def __init__(self):
        super().__init__('jarabot_topic_viewer')
        self.topic_subs = {}
        self.latest_msgs = {}

    def subscribe(self, topic_name, msg_type):
        if topic_name in self.topic_subs:
            self.destroy_subscription(self.topic_subs[topic_name])

        def callback(msg):
            self.latest_msgs[topic_name] = message_to_yaml(msg)

        self.topic_subs[topic_name] = self.create_subscription(
            msg_type, topic_name, callback, QoSProfile(depth=10)
        )


# ==================================================
# Topic Column
# ==================================================
class TopicColumn:
    def __init__(self, parent, node, title, app):
        self.node = node
        self.app = app
        self.current_topic = None
        self.shown_once = False

        self.frame = ttk.Frame(parent)
        lf = ttk.LabelFrame(self.frame, text=title)
        lf.pack(fill='both', expand=True)

        self.var = tk.StringVar()
        self.combo = ttk.Combobox(lf, textvariable=self.var)
        self.combo.pack(fill='x')

        tf = ttk.Frame(lf)
        tf.pack(fill='both', expand=True)

        self.text = tk.Text(tf, wrap='word')
        self.text.pack(side=tk.LEFT, fill='both', expand=True)

        sb = ttk.Scrollbar(tf, orient='vertical', command=self.text.yview)
        sb.pack(side=tk.RIGHT, fill='y')
        self.text.config(yscrollcommand=sb.set)

        self.combo.bind("<<ComboboxSelected>>", self.on_select)

    def set_topics(self, topics):
        self.combo['values'] = topics

    def on_select(self, _=None):
        topic = self.var.get()
        topic_map = dict(self.node.get_topic_names_and_types())
        if topic not in topic_map:
            return
        self.current_topic = topic
        self.shown_once = False
        self.node.subscribe(topic, get_message(topic_map[topic][0]))

    def refresh(self):
        if not self.current_topic:
            return
        if self.current_topic not in self.node.latest_msgs:
            return
        if not self.app.running and self.shown_once:
            return
        self.shown_once = True
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, self.node.latest_msgs[self.current_topic])


# ==================================================
# Topic Tools Window
# ==================================================
class TopicToolsWindow:
    COMMANDS = ["list", "info", "type", "find", "hz", "bw", "delay", "echo"]
    
    def __init__(self, parent, node):
        self.node = node
        self.win = tk.Toplevel(parent)
        self.win.title("Topic Tools")
        self.win.geometry("1020x650")
        
        self.selected_topic = tk.StringVar(value="(none)")
        self.selected_cmd = tk.StringVar(value="(none)")
        
        # 옵션 변수
        self.opt_duration = tk.StringVar(value="3")
        self.opt_window = tk.StringVar(value="10")
        self.opt_echo_n = tk.StringVar(value="1")
        self.opt_list_types = tk.BooleanVar(value=False)
        
        # 내부 상태
        self._topics_cache = []
        self._worker_q = queue.Queue()
        self._running = False
        
        self._build_ui()
        self._poll_worker_queue()
        self.refresh_topics()
    
    def _build_ui(self):
        main_frame = ttk.Frame(self.win)
        main_frame.pack(fill="both", expand=True)
        main_frame.columnconfigure(2, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        # 좌측: Topic / Command
        left = ttk.Frame(main_frame)
        left.grid(row=0, column=0, sticky="nsw", padx=8, pady=8)
        
        # Topic 리스트
        ttk.Label(left, text="Topic").grid(row=0, column=0, sticky="w")
        self.lb_topics = tk.Listbox(left, height=18, width=22, exportselection=False)
        self.lb_topics.grid(row=1, column=0, sticky="w")
        self.lb_topics.bind("<<ListboxSelect>>", self.on_topic_select)
        
        btns = ttk.Frame(left)
        btns.grid(row=2, column=0, sticky="we", pady=(6, 12))
        ttk.Button(btns, text="Refresh", command=self.refresh_topics).pack(side="left")
        
        # Command 리스트
        ttk.Label(left, text="Command").grid(row=3, column=0, sticky="w")
        self.lb_cmds = tk.Listbox(left, height=10, width=22, exportselection=False)
        self.lb_cmds.grid(row=4, column=0, sticky="w")
        for c in self.COMMANDS:
            self.lb_cmds.insert(tk.END, c)
        self.lb_cmds.bind("<<ListboxSelect>>", self.on_cmd_select)
        
        # 가운데: Selection / Options / Preview / Run
        mid = ttk.Frame(main_frame)
        mid.grid(row=0, column=1, sticky="ns", padx=8, pady=8)
        
        # Selection box
        sel = ttk.LabelFrame(mid, text="Selection")
        sel.pack(fill="x", pady=(0, 10))
        
        ttk.Label(sel, text="Selected Topic:").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(sel, textvariable=self.selected_topic, foreground="#1f6feb").grid(row=0, column=1, sticky="w", padx=8, pady=4)
        
        ttk.Label(sel, text="Selected Command:").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Label(sel, textvariable=self.selected_cmd, foreground="#1f6feb").grid(row=1, column=1, sticky="w", padx=8, pady=4)
        
        # Options box
        opt = ttk.LabelFrame(mid, text="Options")
        opt.pack(fill="both", expand=True)
        
        # duration
        ttk.Label(opt, text="Duration (sec)").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.ent_duration = ttk.Entry(opt, textvariable=self.opt_duration, width=8)
        self.ent_duration.grid(row=0, column=1, sticky="w", padx=8, pady=(8, 4))
        self.ent_duration.bind("<KeyRelease>", lambda e: self._update_command_input())
        
        # window
        ttk.Label(opt, text="Window").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.ent_window = ttk.Entry(opt, textvariable=self.opt_window, width=8)
        self.ent_window.grid(row=1, column=1, sticky="w", padx=8, pady=4)
        self.ent_window.bind("<KeyRelease>", lambda e: self._update_command_input())
        
        # echo -n
        ttk.Label(opt, text="Echo count (-n)").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        self.ent_echo_n = ttk.Entry(opt, textvariable=self.opt_echo_n, width=8)
        self.ent_echo_n.grid(row=2, column=1, sticky="w", padx=8, pady=4)
        self.ent_echo_n.bind("<KeyRelease>", lambda e: self._update_command_input())
        
        # list -t
        self.chk_list_types = ttk.Checkbutton(opt, text="list: show types (-t)", variable=self.opt_list_types, command=self._update_command_input)
        self.chk_list_types.grid(row=3, column=0, columnspan=2, sticky="w", padx=8, pady=(6, 8))
        
        # Command Input (editable)
        cmd_input = ttk.LabelFrame(mid, text="Command Input")
        cmd_input.pack(fill="x", pady=(10, 8))
        self.cmd_input = tk.Text(cmd_input, height=2, width=42, wrap="none")
        self.cmd_input.pack(fill="x", padx=8, pady=6)
        self.cmd_input.insert("1.0", "ros2 topic list")
        
        self.btn_run = ttk.Button(mid, text="RUN", command=self.on_run)
        self.btn_run.pack(fill="x", pady=(6, 0))
        
        # 우측: Output
        right = ttk.Frame(main_frame)
        right.grid(row=0, column=2, sticky="nsew", padx=8, pady=8)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        
        self.output = tk.Text(right, wrap="word")
        self.output.grid(row=0, column=0, sticky="nsew")
        sc = ttk.Scrollbar(right, orient="vertical", command=self.output.yview)
        sc.grid(row=0, column=1, sticky="ns")
        self.output.configure(yscrollcommand=sc.set)
        
        self._apply_option_enable_state()
        self._update_command_input()
    
    def _set_command_input(self, s):
        self.cmd_input.delete("1.0", tk.END)
        self.cmd_input.insert("1.0", s)
    
    def _get_command_input(self):
        return self.cmd_input.get("1.0", tk.END).strip()
    
    def _append_output(self, s):
        self.output.insert(tk.END, s)
        self.output.see(tk.END)
    
    def _clear_output(self):
        self.output.delete("1.0", tk.END)
    
    def refresh_topics(self):
        prev_topic = self.selected_topic.get()
        if prev_topic == "(none)":
            prev_topic = None
        
        topics = [t for t, _ in self.node.get_topic_names_and_types()]
        
        self._topics_cache = topics
        self.lb_topics.delete(0, tk.END)
        for t in topics:
            self.lb_topics.insert(tk.END, t)
        
        if prev_topic and prev_topic in topics:
            idx = topics.index(prev_topic)
            self.lb_topics.selection_set(idx)
            self.lb_topics.see(idx)
            self.selected_topic.set(prev_topic)
        else:
            self.selected_topic.set("(none)")
        
        self._update_command_input()
    
    def on_topic_select(self, _evt=None):
        idxs = self.lb_topics.curselection()
        if not idxs:
            self.selected_topic.set("(none)")
        else:
            self.selected_topic.set(self.lb_topics.get(idxs[0]))
        self._update_command_input()
    
    def on_cmd_select(self, _evt=None):
        idxs = self.lb_cmds.curselection()
        if not idxs:
            self.selected_cmd.set("(none)")
        else:
            self.selected_cmd.set(self.lb_cmds.get(idxs[0]))
        self._apply_option_enable_state()
        self._update_command_input()
    
    def _apply_option_enable_state(self):
        cmd = self.selected_cmd.get()
        
        def set_state(w, enabled):
            w.configure(state=("normal" if enabled else "disabled"))
        
        self.chk_list_types.configure(state=("normal" if cmd == "list" else "disabled"))
        set_state(self.ent_duration, cmd in ("hz", "bw", "delay"))
        set_state(self.ent_window, cmd in ("hz", "bw"))
        set_state(self.ent_echo_n, cmd == "echo")
    
    def _update_command_input(self):
        cmd = self.selected_cmd.get()
        topic = self.selected_topic.get()
        
        if cmd == "(none)":
            self._set_command_input("ros2 topic list")
            return
        
        built = self._build_ros2_cmd(cmd, topic, preview_only=True)
        if built:
            self._set_command_input(" ".join(built))
        else:
            self._set_command_input("# Select a topic first")
    
    def _safe_int(self, s, default):
        try:
            return int(str(s).strip())
        except:
            return default
    
    def _safe_float(self, s, default):
        try:
            return float(str(s).strip())
        except:
            return default
    
    def _build_ros2_cmd(self, cmd, topic, preview_only=False):
        need_topic = cmd in ("info", "type", "hz", "bw", "delay", "echo")
        if need_topic and (not topic or topic == "(none)"):
            return []
        
        base = ["ros2", "topic", cmd]
        
        if cmd == "list":
            if self.opt_list_types.get():
                base.append("-t")
            return base
        
        if cmd == "info":
            return base + [topic]
        
        if cmd == "type":
            return base + [topic]
        
        if cmd == "find":
            if preview_only:
                return ["ros2", "topic", "find", "<msg_type (auto from selected topic)>"]
            return ["__FIND_AUTO__", topic]
        
        if cmd == "hz":
            dur = self._safe_float(self.opt_duration.get(), 3.0)
            win = self._safe_int(self.opt_window.get(), 10)
            return base + [topic, "-d", str(dur), "-w", str(win)]
        
        if cmd == "bw":
            dur = self._safe_float(self.opt_duration.get(), 3.0)
            win = self._safe_int(self.opt_window.get(), 10)
            return base + [topic, "-d", str(dur), "-w", str(win)]
        
        if cmd == "delay":
            dur = self._safe_float(self.opt_duration.get(), 3.0)
            return base + [topic, "-d", str(dur)]
        
        if cmd == "echo":
            n = self._safe_int(self.opt_echo_n.get(), 1)
            return base + [topic, "-n", str(n)]
        
        return base
    
    def on_run(self):
        if self._running:
            messagebox.showinfo("Running", "A command is already running.")
            return
        
        # Get command from input field
        cmd_text = self._get_command_input()
        
        if not cmd_text or cmd_text.startswith("#"):
            messagebox.showwarning("Empty Command", "Enter a valid command to execute.")
            return
        
        # Parse command string into list
        import shlex
        try:
            cmd_list = shlex.split(cmd_text)
        except Exception as e:
            messagebox.showerror("Parse Error", f"Failed to parse command:\n{e}")
            return
        
        if not cmd_list:
            messagebox.showwarning("Empty Command", "Enter a valid command to execute.")
            return
        
        self._clear_output()
        self._append_output(f"$ {cmd_text}\n\n")
        
        self._running = True
        self.btn_run.configure(state="disabled")
        
        # Determine timeout based on command type
        timeout = None
        cmd_name = self.selected_cmd.get()
        if cmd_name in ("hz", "bw", "delay"):
            dur = self._safe_float(self.opt_duration.get(), 3.0)
            timeout = max(1.0, dur + 2.0)
        elif cmd_name == "echo":
            timeout = 6
        
        th = threading.Thread(target=self._worker_run_direct, args=(cmd_list, timeout), daemon=True)
        th.start()
    
    def _worker_run_direct(self, cmd_list, timeout):
        """Execute command directly from command input field"""
        rc, out, err = run_cmd(cmd_list, timeout=timeout)
        text = out
        if err:
            text += ("\n" if text and not text.endswith("\n") else "") + err
        if not text.strip():
            text = "(no output)\n"
        self._worker_q.put(("done", text))
    
    def _worker_run(self, cmd, topic, built):
        if built and built[0] == "__FIND_AUTO__":
            rc1, out1, err1 = run_cmd(["ros2", "topic", "type", topic], timeout=3)
            msg_type = out1.strip() if rc1 == 0 else ""
            if not msg_type:
                self._worker_q.put(("done", f"[error] cannot get topic type.\n{err1}\n"))
                return
            rc, out, err = run_cmd(["ros2", "topic", "find", msg_type], timeout=3)
            self._worker_q.put(("done", out + (("\n" + err) if err else "")))
            return
        
        timeout = None
        if cmd in ("hz", "bw", "delay"):
            dur = self._safe_float(self.opt_duration.get(), 3.0)
            timeout = max(1.0, dur + 2.0)
        elif cmd == "echo":
            timeout = 6
        
        rc, out, err = run_cmd(built, timeout=timeout)
        text = out
        if err:
            text += ("\n" if text and not text.endswith("\n") else "") + err
        if not text.strip():
            text = "(no output)\n"
        self._worker_q.put(("done", text))
    
    def _poll_worker_queue(self):
        try:
            while True:
                tag, payload = self._worker_q.get_nowait()
                if tag == "done":
                    self._append_output(payload)
                    self._running = False
                    self.btn_run.configure(state="normal")
        except queue.Empty:
            pass
        self.win.after(80, self._poll_worker_queue)


# ==================================================
# Main App
# ==================================================
class TopicViewerApp:
    def __init__(self, root):
        self.root = root
        root.title(f"{APP_NAME} {APP_VERSION}")
        root.geometry("1100x650")
        root.minsize(900, 500)

        rclpy.init()
        self.node = TopicViewerNode()
        self.running = False

        self.create_menu()
        self.create_toolbar()
        self.create_columns()

        self.update_topics()
        self.update_ui()
        threading.Thread(target=self.spin_ros, daemon=True).start()

    # ---------------- Menu ----------------
    def create_menu(self):
        mb = tk.Menu(self.root)
        mb.add_command(label="Topic Structure", command=self.topic_structure)
        mb.add_command(label="Topic Tools", command=self.topic_tools)
        mb.add_command(label="About", command=self.about)
        self.root.config(menu=mb)

    # ---------------- Toolbar ----------------
    def create_toolbar(self):
        bar = ttk.Frame(self.root)
        bar.pack(fill='x', pady=4)

        tk.Button(
            bar, text="START", bg="#2ecc71", fg="white",
            activebackground="#27ae60", width=10,
            command=lambda: self.set_run(True)
        ).pack(side=tk.LEFT, padx=5)

        tk.Button(
            bar, text="STOP", bg="#e74c3c", fg="white",
            activebackground="#c0392b", width=10,
            command=lambda: self.set_run(False)
        ).pack(side=tk.LEFT, padx=5)

    def set_run(self, v):
        self.running = v
        if not v:
            for c in self.columns:
                c.shown_once = False

    # ---------------- Columns ----------------
    def create_columns(self):
        pw = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        pw.pack(fill='both', expand=True)

        self.columns = [
            TopicColumn(pw, self.node, "Topic1", self),
            TopicColumn(pw, self.node, "Topic2", self),
            TopicColumn(pw, self.node, "Topic3", self),
        ]
        for c in self.columns:
            pw.add(c.frame, weight=1)

    # ---------------- Update ----------------
    def update_topics(self):
        topics = [t for t, _ in self.node.get_topic_names_and_types()]
        for c in self.columns:
            c.set_topics(topics)
        self.root.after(2000, self.update_topics)

    def update_ui(self):
        for c in self.columns:
            c.refresh()
        self.root.after(100, self.update_ui)

    def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ==================================================
    # Topic Structure
    # ==================================================
    def topic_structure(self):
        win = tk.Toplevel(self.root)
        win.title("Topic Structure")
        win.geometry("900x600")

        lb = tk.Listbox(win, width=32)
        lb.pack(side=tk.LEFT, fill='y')

        txt = tk.Text(win, wrap='word')
        txt.pack(side=tk.RIGHT, fill='both', expand=True)

        topics = dict(self.node.get_topic_names_and_types())
        for t in topics:
            lb.insert(tk.END, t)

        def show(_):
            txt.delete("1.0", tk.END)
            sel = lb.curselection()
            if not sel:
                return
            topic = lb.get(sel[0])
            msg_type = topics[topic][0]

            txt.insert(tk.END, f"Topic : {topic}\n")
            txt.insert(tk.END, f"Type  : {msg_type}\n\n")

            ok, out = run_cmd_capture(["ros2", "interface", "show", msg_type])
            txt.insert(tk.END, out if out else ("(no output)" if ok else "(error)"))

        lb.bind("<<ListboxSelect>>", show)

    # ==================================================
    # Topic Tools
    # ==================================================
    def topic_tools(self):
        TopicToolsWindow(self.root, self.node)

    # ---------------- About ----------------
    def about(self):
        messagebox.showinfo(
            "About",
            f"{APP_NAME}\n\n"
            f"Version : {APP_VERSION}\n"
            f"Author  : {APP_AUTHOR}\n\n"
            "Advanced ROS2 Topic Debugging & Education Tool\n"
            "with Integrated Topic Tools"
        )


# ==================================================
# Entry
# ==================================================
def main():
    root = tk.Tk()
    TopicViewerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
