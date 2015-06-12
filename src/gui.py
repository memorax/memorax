#! %%PYTHON%%

## Copyright (C) 2012 Carl Leonardsson
## 
## This file is part of Memorax.
##
## Memorax is free software: you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
## 
## Memorax is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
## or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
## License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

import Tkinter
import tkFileDialog
import subprocess
import sys
import string
import time
import thread
import json
import os

class MainWindow:        

    class Conf:
        # The configuration file
        conffilename = os.path.expanduser("~/.memorax-gui")
        # Path to the binary of the back-end command-line tool
        binary="%%BINARY%%"
        # Last directory used for loading/storing rmm code
        last_code_dir="."
        # Command to use for opening PDF files
        pdf_viewer=""
        # Should the configuration be saved to file?
        do_save_to_file=False

        def __init__(self):
            if os.path.exists(self.conffilename):
                self.read_conf(self.conffilename)
                self.do_save_to_file = True

        def read_conf(self,conffilename):
            try:
                f = open(conffilename)
                s = f.read()
                try:
                    cfg = json.loads(s)
                    for k in cfg.keys():
                        if k == "binary":
                            if type(cfg[k]) == unicode or type(cfg[k]) == str:
                                self.binary = cfg[k]
                            else:
                                print "Error in configuration file: "+ \
                                "Value of key 'binary' should be a file path."
                        elif k == "lastcodedir":
                            if type(cfg[k]) == unicode or type(cfg[k]) == str:
                                self.last_code_dir = cfg[k]
                            else:
                                print "Error in configuration file: "+ \
                                "Value of key 'lastcodedir' should be a file path."
                        elif k == "pdfviewer":
                            if type(cfg[k]) == unicode or type(cfg[k]) == str:
                                self.pdf_viewer = cfg[k]
                            else:
                                print "Error in configuration file: "+ \
                                "Value of key 'pdfviewer' should be a file path."
                        else:
                            print "Ignoring unknown configuration key '"+k+"'."
                except:
                    print "Invalid configuration file."
                    print "Please make sure that the configuration file "+ \
                    "has correct JSON syntax."
                    print sys.exc_info()[1]
            except:
                print "Failed to open configuration file",conffilename
            finally:
                try:
                    f.close()
                except:
                    pass

        def write_conf(self,conffilename):
            try:
                f = open(conffilename,mode="w")
                f.write(json.dumps({
                            'binary':self.binary,
                            'lastcodedir':self.last_code_dir,
                            'pdfviewer':self.pdf_viewer
                            }))
            except:
                print "Failed to save configuration to file",conffilename
                print sys.exc_info()[1]
            finally:
                try:
                    f.close()
                except:
                    pass

        def write_conf_default(self):
            if self.do_save_to_file:
                self.write_conf(self.conffilename)

    class ConfDialog:
        def __init__(self,conf,master):
            self.conf = conf
            self.master = master
            self.dialog = Tkinter.Toplevel(self.master)
            self.dialog.title("Configuration")

            main_frame = Tkinter.Frame(self.dialog)
            main_frame.pack(fill=Tkinter.BOTH,expand=1)

            # Binary
            bin_frame = Tkinter.Frame(main_frame)
            bin_frame.pack(side=Tkinter.TOP)
            Tkinter.Label(bin_frame,text="Path to command-line tool:").pack(side=Tkinter.LEFT)
            self.bin_entry = Tkinter.Entry(bin_frame)
            self.bin_entry.insert("end",self.conf.binary)
            self.bin_entry.pack(side=Tkinter.LEFT)
            bin_btn = Tkinter.Button(bin_frame,text="Choose",command=self.choose_binary)
            bin_btn.pack(side=Tkinter.RIGHT)

            # PDF viewer
            pdf_frame = Tkinter.Frame(main_frame)
            pdf_frame.pack(side=Tkinter.TOP)
            Tkinter.Label(pdf_frame,text="PDF viewer:").pack(side=Tkinter.LEFT)
            self.pdf_entry = Tkinter.Entry(pdf_frame)
            self.pdf_entry.insert("end",self.conf.pdf_viewer)
            self.pdf_entry.pack(side=Tkinter.LEFT)
            pdf_btn = Tkinter.Button(pdf_frame,text="Choose",command=self.choose_pdf)
            pdf_btn.pack(side=Tkinter.RIGHT)

            # Save checkbox
            save_check_frame = Tkinter.Frame(main_frame)
            save_check_frame.pack(side=Tkinter.TOP)
            self.save_check = Tkinter.IntVar()
            self.save_check.set(conf.do_save_to_file)
            wg_save_check = Tkinter.Checkbutton(save_check_frame,text="Save configuration to home directory (~/.memorax-gui)",
                                                variable=self.save_check)
            wg_save_check.pack()

            # Ok
            ok_frame = Tkinter.Frame(main_frame)
            ok_frame.pack(side=Tkinter.TOP, fill=Tkinter.X, expand=1)
            ok_btn = Tkinter.Button(ok_frame,text="Ok",command=self.ok)
            ok_btn.pack(side=Tkinter.RIGHT)

        def choose_file(self,entry,title):
            filename = tkFileDialog.askopenfilename(initialdir="/usr/bin",
                                                    parent=self.master,
                                                    title=title)
            if len(filename) > 0:
                entry.delete(0,"end")
                entry.insert("end",filename)

        def choose_binary(self):
            self.choose_file(self.bin_entry,"Choose Memorax binary")

        def choose_pdf(self):
            self.choose_file(self.pdf_entry,"Choose PDF viewer")

        def ok(self):
            self.conf.binary = self.bin_entry.get()
            self.conf.pdf_viewer = self.pdf_entry.get()
            self.conf.do_save_to_file = self.save_check.get()
            self.dialog.destroy()

    def open_conf_dialog(self):
        self.conf_dialog = self.ConfDialog(self.conf,self.master)

    start_code="/* An example code */\n\n\
forbidden\n\
  CS CS\n\
\n\
data\n\
  x = 0 : [0:1]\n\
  y = 0 : [0:1]\n\
\n\
process\n\
text\n\
L0:\n\
  write: x := 1;\n\
  read: y = 0;\n\
CS:\n\
  write: x := 0;\n\
  goto L0\n\
\n\
process\n\
text\n\
L0:\n\
  write: y := 1;\n\
  read: x = 0;\n\
CS:\n\
  write: y := 0;\n\
  goto L0"

    def __init__(self, master):
        self.subthread_count = 0
        self.subthread_count_lock = thread.allocate_lock()
        self.conf = self.Conf()

        self.master = master

        self.bg_colour = "#ff9100"

        self.wg_mainframe = Tkinter.Frame(master,bg=self.bg_colour)
        self.wg_mainframe.pack(fill=Tkinter.BOTH,expand=1)
        self.fence_links = []

        # Menu bar
        self.wg_menubar = Tkinter.Menu(master)
        self.wg_file_menu = Tkinter.Menu(self.wg_menubar,tearoff=0)
        self.wg_file_menu.add_command(label="New",command=self.new_file,
                                      underline=0)
        self.wg_file_menu.add_command(label="Open",command=self.open_file,
                                      accelerator="Ctrl+O",underline=0)
        self.wg_file_menu.add_command(label="Save",command=self.save_file,
                                      accelerator="Ctrl+S",underline=0)
        self.wg_file_menu.add_command(label="Save as",command=self.save_file_as,
                                      underline=1)
        self.wg_file_menu.add_command(label="Save output as",command=self.save_output_as,
                                      underline=6)
        self.wg_file_menu.add_separator()
        self.wg_file_menu.add_command(label="Quit",command=self.quit,
                                      accelerator="Ctrl+Q",underline=0)
        self.wg_menubar.add_cascade(label="File",menu=self.wg_file_menu,underline=0)
        self.wg_misc_menu = Tkinter.Menu(self.wg_menubar,tearoff=0)
        self.wg_misc_menu.add_command(label="Configuration",command=self.open_conf_dialog,
                                      underline=0)
        self.wg_menubar.add_cascade(label="Misc",menu=self.wg_misc_menu,underline=0)

        master.config(menu=self.wg_menubar)
        master.bind_all("<Control-o>",lambda evt: self.open_file())
        master.bind_all("<Control-s>",lambda evt: self.save_file())
        master.bind_all("<Control-q>",lambda evt: self.quit())
        master.bind_all("<Control-O>",lambda evt: self.open_file())
        master.bind_all("<Control-S>",lambda evt: self.save_file())
        master.bind_all("<Control-Q>",lambda evt: self.quit())
        master.bind_all("<Control-c>",lambda evt: self.opt_interrupt())
        master.bind_all("<Control-C>",lambda evt: self.opt_interrupt())
        master.bind_all("<Control-r>",lambda evt: self.opt_run())
        master.bind_all("<Control-R>",lambda evt: self.opt_run())
        master.bind("<Destroy>",self.destroy_evt)

        # Command selection
        self.wg_command_frame = Tkinter.Frame(self.wg_mainframe,bg=self.bg_colour,borderwidth=2,relief=Tkinter.RAISED)
        self.wg_command_frame.pack(side=Tkinter.TOP,fill=Tkinter.X);

        self.commands = {
            0:"reach",
            1:"fencins",
            2:"dotify"
            }
        self.command_sel = Tkinter.IntVar() # The main command sel
        self.wg_command_sel_frame = Tkinter.Frame(self.wg_command_frame,bg=self.bg_colour)
        self.wg_command_sel_frame.pack(side=Tkinter.TOP)
        self.wg_command_sel_reach = Tkinter.Radiobutton(self.wg_command_sel_frame,text="Reachability",
                                                        variable=self.command_sel,value=0,bg=self.bg_colour,
                                                        command=self.select_command_from_wg)
        self.wg_command_sel_reach.pack(side=Tkinter.LEFT)
        self.set_hint(self.wg_command_sel_reach,
                      "Analyse reachability of forbidden control states. (Shortcut: F1)")
        self.wg_command_sel_fencins = Tkinter.Radiobutton(self.wg_command_sel_frame,text="Fence insertion",
                                                          variable=self.command_sel,value=1,bg=self.bg_colour,
                                                          command=self.select_command_from_wg)
        self.wg_command_sel_fencins.pack(side=Tkinter.LEFT)
        self.set_hint(self.wg_command_sel_fencins,
                      "Automatically insert fences to guarantee unreachability of\nforbidden control states. (Shortcut: F2)")
        self.wg_command_sel_dotify = Tkinter.Radiobutton(self.wg_command_sel_frame,text="Draw automata",
                                                         variable=self.command_sel,value=2,bg=self.bg_colour,
                                                         command=self.select_command_from_wg)
        self.wg_command_sel_dotify.pack(side=Tkinter.LEFT)
        self.set_hint(self.wg_command_sel_dotify,
                      "Generate and display the automaton corresponding to the RMM\ncode in PDF format. (Shortcut: F3)")
        self.wg_command_mid_frame = Tkinter.Frame(self.wg_command_frame,bg=self.bg_colour,borderwidth=1,relief=Tkinter.RIDGE)
        self.wg_command_mid_frame.pack(side=Tkinter.TOP,fill=Tkinter.X)
        self.wg_command_bottom_frame = Tkinter.Frame(self.wg_command_frame,bg=self.bg_colour)
        self.wg_command_bottom_frame.pack(side=Tkinter.BOTTOM)
        self.verbosity_sel = Tkinter.StringVar()
        self.verbosity_sel.set("Messages")
        self.wg_verbosity_frame = Tkinter.Frame(self.wg_command_bottom_frame,bg=self.bg_colour)
        self.wg_verbosity_frame.pack(side=Tkinter.LEFT)
        Tkinter.Label(self.wg_verbosity_frame,text="Verbosity level:",bg=self.bg_colour).pack(side=Tkinter.TOP)
        self.wg_verbosity_sel = Tkinter.OptionMenu(self.wg_verbosity_frame,self.verbosity_sel,
                                                   "Only Results","Messages","Debug","Extreme")
        self.wg_verbosity_sel.pack(side=Tkinter.BOTTOM)
        self.set_hint(self.wg_verbosity_sel,
                      "Specify the level of verbosity of the output.")
        self.wg_interrupt_btn = Tkinter.Button(self.wg_command_bottom_frame,text="Break",
                                               command=self.interrupt,state=Tkinter.DISABLED)
        self.wg_interrupt_btn.pack(side=Tkinter.RIGHT)
        self.set_hint(self.wg_interrupt_btn, "Interrupt a running analysis. (Can also be called by Ctrl+C)")
        self.wg_run_btn = Tkinter.Button(self.wg_command_bottom_frame,text="Run",command=self.run)
        self.set_hint(self.wg_run_btn,"Start running the command. (Can also be called by Ctrl+R)")
        self.wg_run_btn.pack(side=Tkinter.RIGHT)
        self.command_sel_widgets = []
        self.abs_sel = Tkinter.StringVar() # selects abstraction
        self.abs_sel.set("sb")
        self.cegar_check = Tkinter.IntVar() # Checks whether or not to use CEGAR
        self.cegar_check.set(True)
        self.rff_check = Tkinter.IntVar() # Checks whether or not to use register free form
        self.rff_check.set(True)
        self.fence_check = Tkinter.IntVar() # Checks whether or not to use register free form
        self.fence_check.set(False)

        master.bind_all("<F1>",lambda evt: self.select_command("reach"))
        master.bind_all("<F2>",lambda evt: self.select_command("fencins"))
        master.bind_all("<F3>",lambda evt: self.select_command("dotify"))

        # Hint
        self.wg_hint = Tkinter.Label(self.wg_mainframe,bg="white",text="",anchor=Tkinter.W,justify=Tkinter.LEFT,
                                     borderwidth=2,relief=Tkinter.RIDGE,font="* 12 bold")
        self.wg_hint.pack(side=Tkinter.BOTTOM,fill=Tkinter.X)

        # Panes
        self.wg_paned_window = Tkinter.PanedWindow(self.wg_mainframe,bg=self.bg_colour,orient=Tkinter.VERTICAL,
                                                   showhandle=True,sashrelief=Tkinter.RIDGE)
        self.wg_paned_window.pack(fill=Tkinter.BOTH,expand=1)

        # Code
        self.wg_code_frame = Tkinter.LabelFrame(self.wg_paned_window,text="(Not Saved)")
        self.wg_code_frame.pack(side=Tkinter.TOP,fill=Tkinter.BOTH,expand=1)
        self.wg_code = Tkinter.Text(self.wg_code_frame,height=15,
                                    font="courier 10 bold", tabs=2)
        self.wg_code.insert("1.0",self.start_code)
        self.wg_code_sb = Tkinter.Scrollbar(self.wg_code_frame,command=self.wg_code.yview)
        self.wg_code_sb.pack(side=Tkinter.RIGHT,fill=Tkinter.Y)
        self.wg_code.pack(side=Tkinter.LEFT,fill=Tkinter.BOTH,expand=1)
        self.wg_code.config(yscrollcommand=self.wg_code_sb.set)
        self.wg_code.edit_modified(True)
        self.code_filename = None
        self.show_filename()
        self.wg_code.bind("<KeyRelease>",self.code_event)
        self.wg_paned_window.add(self.wg_code_frame)

        # Output
        self.wg_output_frame = Tkinter.Frame(self.wg_paned_window,bg=self.bg_colour)
        self.wg_output_frame.pack(side=Tkinter.BOTTOM,expand=1,fill=Tkinter.BOTH)
        self.wg_output_frame_top = Tkinter.Frame(self.wg_output_frame,bg=self.bg_colour)
        self.wg_output_frame_top.pack(side=Tkinter.TOP,fill=Tkinter.X)
        self.output_sel = Tkinter.StringVar()
        self.output_sel.set("output")
        self.wg_output_sel_output = Tkinter.Radiobutton(self.wg_output_frame_top,text="Output",
                                                        variable=self.output_sel,value="output",
                                                        indicatoron=0,command=self.select_output_from_wg)
        self.wg_output_sel_output.pack(side=Tkinter.LEFT)
        self.wg_output_sel_error = Tkinter.Radiobutton(self.wg_output_frame_top,text="Error",
                                                       variable=self.output_sel,value="error",
                                                       indicatoron=0,command=self.select_output_from_wg)
        self.wg_output_sel_error.pack(side=Tkinter.LEFT)
        self.wg_running_lbl = Tkinter.Label(self.wg_output_frame_top,text="",font="courier 10 bold",
                                            bg=self.bg_colour)
        self.wg_running_lbl.pack(side=Tkinter.RIGHT)
        self.wg_output_frame_bottom = Tkinter.Frame(self.wg_output_frame)
        self.wg_output_frame_bottom.pack(side=Tkinter.BOTTOM,fill=Tkinter.BOTH,expand=1)
        self.wg_output = Tkinter.Text(self.wg_output_frame_bottom,height=10,
                                      state=Tkinter.DISABLED)
        self.wg_output.pack(side=Tkinter.LEFT,fill=Tkinter.BOTH,expand=1)
        self.output_text=""
        self.error_text=""
        self.wg_output_sb = Tkinter.Scrollbar(self.wg_output_frame_bottom,command=self.wg_output.yview)
        self.wg_output_sb.pack(side=Tkinter.RIGHT,fill=Tkinter.Y);
        self.wg_output.config(yscrollcommand=self.wg_output_sb.set)
        self.wg_paned_window.add(self.wg_output_frame)

        self.output_lock = thread.allocate_lock()
        self.select_command("reach")
        self.set_running(False)

        self.poutput_version()

        if(len(sys.argv) > 1):
            self.open_file_by_name(sys.argv[1])
            if(len(sys.argv) > 2):
                self.pwarning("Ignoring command line arguments:\n")
                for a in sys.argv[2:]:
                    self.pwarning("  {0}\n".format(a))
            

    def code_event(self,evt):
        self.show_filename()
        self.clear_error_marks_in_code()
        self.unset_highlights_in_code()

    def set_hint(self,widget,text):
        widget.hint_text = text
        widget.bind("<Enter>",self.show_hint)
        widget.bind("<Leave>",self.hide_hint)

    def show_hint(self,evt):
        self.wg_hint.config(text=evt.widget.hint_text)

    def hide_hint(self,evt):
        self.wg_hint.config(text="")

    def select_output_from_wg(self):
        self.select_output(self.output_sel.get())

    def select_output(self,val):
        self.output_sel.set(val)
        if val == "output":
            self.wg_output.config(bg="white",fg="black")
        else:
            self.wg_output.config(bg="black",fg="red")
        self.update_output()

    def update_output(self):
        self.output_lock.acquire()
        self.wg_output.config(state=Tkinter.NORMAL)
        self.wg_output.delete("1.0","end")
        if self.output_sel.get() == "output":
            self.wg_output.insert("end",self.output_text)
        else:
            self.wg_output.insert("end",self.error_text)
        self.wg_output.see("end")
        self.wg_output.config(state=Tkinter.DISABLED)
        self.update_fence_link_marks()
        self.output_lock.release()

    def perror(self,s):
        self.error_text += s
        if len(s):
            n = len(self.error_text.split("\n"))
            if self.error_text.endswith("\n"): n = n-1
            self.wg_output_sel_error.config(text="Error ({0})".format(n),
                                            fg="red")
        self.update_output()
        self.select_output("error")

    def pwarning(self,s):
        self.error_text += s
        if len(s):
            n = len(self.error_text.split("\n"))
            if self.error_text.endswith("\n"): n = n-1
            self.wg_output_sel_error.config(text="Error ({0})".format(n),
                                            fg="red")
        self.update_output()

    def clear_errors(self):
        self.wg_output_sel_error.config(text="Error",
                                        fg="black")
        self.error_text = ""
        self.update_output()

    def perror_clear(self,s):
        self.clear_errors()
        self.perror(s)
        self.update_output()

    def poutput(self,s):
        self.output_text += s
        self.update_output()

    def clear_output(self):
        self.output_text = ""
        self.update_output()

    def poutput_clear(self,s):
        self.clear_output()
        self.poutput(s)
        self.update_output()

    def poutput_version(self):
        v = """%%GUI_STRING%%
Copyright (C) 2012 Carl Leonardsson
This program comes with ABSOLUTELY NO WARRANTY. This is free software and you
are welcome to redistribute it under certain conditions. See the full text of
the GNU General Public License Version 3 (http://www.gnu.org/licenses/).\n"""
        self.poutput(v)

    def show_filename(self):
        if self.code_filename == None:
            self.wg_code_frame.config(text="(Not saved)")
        else:
            if self.wg_code.edit_modified():
                self.wg_code_frame.config(text=self.code_filename+"*")
            else:
                self.wg_code_frame.config(text=self.code_filename)

    def set_code_filename(self,name):
        self.code_filename=name
        self.code_event(None)
        self.set_dotify_output_from_code_filename()

    def get_code_filename(self):
        return self.code_filename

    def shortcut(self,evt):
        print "shortcut \\o/"

    def quit(self):
        self.cleanup()
        self.master.quit()

    def new_file(self):
        self.wg_code.delete("1.0","end")
        self.wg_code.edit_modified(False)
        self.set_code_filename(None)
        self.original_code = ""

    def open_file_by_name(self,filename):
        try:
            f = open(filename)
            lns = f.readlines()
            code = ""
            for l in lns:
                code += l
            self.wg_code.delete("1.0","end")
            self.wg_code.insert("end",code)
            self.wg_code.edit_modified(False)
            self.set_code_filename(filename)
            self.original_code = self.wg_code.get("1.0","end")
            f.close()
            self.conf.last_code_dir = os.path.dirname(filename)
        except:
            self.perror_clear(str(sys.exc_info()[1])+"\n")
            try:
                f.close()
            except:
                pass

    def open_file(self):
        filename = tkFileDialog.askopenfilename(filetypes=[('RMM Files','.rmm'),('All Files','.*')],
                                                initialdir=self.conf.last_code_dir,
                                                title="Open RMM code file",
                                                parent=self.master)
        if len(filename) > 0:
            self.open_file_by_name(filename)

    def save_in_file(self,filename):
        try:
            f = open(filename,mode="w")
            f.write(self.wg_code.get("1.0","end"))
            f.close()
            self.set_code_filename(filename)
            self.wg_code.edit_modified(False)
            self.original_code = self.wg_code.get("1.0","end")
        except:
            self.perror_clear(str(sys.exc_info()[1])+"\n")
            try:
                f.close()
            except:
                pass

    def save_file(self):
        if self.get_code_filename() == None:
            self.save_file_as()
        else:
            self.save_in_file(self.get_code_filename())
        self.show_filename()

    def save_file_as(self):
        filename = tkFileDialog.asksaveasfilename(filetypes=[('RMM Files','.rmm'),('All Files','.*')],
                                                  initialdir=self.conf.last_code_dir,
                                                  title="Save to RMM code file",
                                                  parent=self.master)
        if len(filename) > 0:
            self.conf.last_code_dir = os.path.dirname(filename)
            self.save_in_file(filename)
        self.show_filename()

    def save_output_as(self):
        filename = tkFileDialog.asksaveasfilename(initialdir=self.conf.last_code_dir,
                                                  title="Save output to file",
                                                  parent=self.master)
        if filename:
            try:
                f = open(filename,mode="w")
                f.write(self.wg_output.get("1.0","end"))
                self.poutput("Wrote output to "+filename+"\n")
            except:
                self.perror_clear("Failed to save output to file.\n"+str(sys.exc_info()[1])+"\n")
            finally:
                try: f.close()
                except: pass

    def clear_command(self):
        for w in self.command_sel_widgets:
            w.destroy()
        self.command_sel_widgets = []

    def select_command(self,cmd):
        self.clear_command()
        for k in self.commands.keys():
            if self.commands[k] == cmd:
                self.command_sel.set(k)
        if cmd == "reach" or cmd == "fencins":
            self.wg_command_inner_frame = Tkinter.Frame(self.wg_command_mid_frame,bg=self.bg_colour)
            self.wg_command_inner_frame.pack()
            self.wg_abs_sel_frame = Tkinter.LabelFrame(self.wg_command_inner_frame,bg=self.bg_colour,text="Abstraction")
            self.wg_abs_sel_frame.pack(side=Tkinter.LEFT)
            self.wg_abs_sel_pb = Tkinter.Radiobutton(self.wg_abs_sel_frame,text="PB",variable=self.abs_sel,value="pb",
                                                     bg=self.bg_colour,command=self.select_abstraction_from_wg)
            self.wg_abs_sel_pb.pack(side=Tkinter.LEFT)
            self.set_hint(self.wg_abs_sel_pb,
                          "Abstraction PB:\nOver approximation storing for a buffer only a finite prefix.\n"+
                          "Uses predicate abstraction.\nMemory model: TSO\nIntegers: Unbounded")
            self.wg_abs_sel_sb = Tkinter.Radiobutton(self.wg_abs_sel_frame,text="SB",variable=self.abs_sel,value="sb",
                                                     bg=self.bg_colour,command=self.select_abstraction_from_wg)
            self.wg_abs_sel_sb.pack(side=Tkinter.LEFT)
            self.set_hint(self.wg_abs_sel_sb,
                          "Abstraction SB:\nExact analysis using the WQO framework over the SB semantic.\n"+
                          "Memory model: TSO (SB)\nIntegers: Bounded")
            self.wg_abs_sel_hsb = Tkinter.Radiobutton(self.wg_abs_sel_frame,text="HSB",variable=self.abs_sel,value="hsb",
                                                      bg=self.bg_colour,command=self.select_abstraction_from_wg)
            self.wg_abs_sel_hsb.pack(side=Tkinter.LEFT)
            self.set_hint(self.wg_abs_sel_hsb,
                          "Abstraction HSB:\nExact analysis using the WQO framework over the HSB semantic.\n"+
                          "Memory model: PSO (SB)\nIntegers: Bounded")
            self.wg_rff_check = Tkinter.Checkbutton(self.wg_command_inner_frame,text="Register Free Form",variable=self.rff_check,
                                                    bg=self.bg_colour)
            self.set_hint(self.wg_rff_check,
                          "Transform the machine to Register Free Form before using it.")
            self.wg_rff_check.pack(side=Tkinter.RIGHT)
            self.wg_cegar_check = Tkinter.Checkbutton(self.wg_command_inner_frame,text="Use CEGAR",variable=self.cegar_check,
                                                      bg=self.bg_colour)
            if self.abstraction_has_cegar(self.abs_sel.get()):
                self.wg_cegar_check.config(state=Tkinter.NORMAL)
            else:
                self.wg_cegar_check.config(state=Tkinter.DISABLED)
            self.set_hint(self.wg_cegar_check,
                          "Use CEGAR for predicate abstraction?\nOtherwise find predicates in RMM code branch conditions.")
            self.wg_cegar_check.pack(side=Tkinter.RIGHT)
            self.command_sel_widgets = [self.wg_abs_sel_frame, self.wg_abs_sel_pb, self.wg_abs_sel_sb,
                                        self.wg_command_inner_frame,self.wg_cegar_check,self.wg_rff_check]
        elif cmd == "dotify":
            self.wg_dotify_frame = Tkinter.Frame(self.wg_command_mid_frame,bg=self.bg_colour)
            self.wg_dotify_frame.pack()
            self.wg_rff_check = Tkinter.Checkbutton(self.wg_dotify_frame,text="Register Free Form",variable=self.rff_check,
                                                    bg=self.bg_colour,command=self.dotify_rff_change)
            self.set_hint(self.wg_rff_check,
                          "Transform the machine to Register Free Form before using it.")
            self.wg_rff_check.pack(side=Tkinter.RIGHT)
            self.wg_fence_check = Tkinter.Checkbutton(self.wg_dotify_frame,text="Use fences",variable=self.fence_check,
                                                    bg=self.bg_colour)
            self.set_hint(self.wg_fence_check,
                          "Represent locked writes with separate fence transitions. This is the form used by HSB.")
            self.wg_fence_check.pack(side=Tkinter.RIGHT)
            self.wg_dotify_lbl = Tkinter.Label(self.wg_dotify_frame,text="Output: ",bg=self.bg_colour)
            self.wg_dotify_output = Tkinter.Entry(self.wg_dotify_frame,width=40)
            self.set_dotify_output_from_code_filename()
            self.wg_dotify_output_btn = Tkinter.Button(self.wg_dotify_frame,text="Choose",command=self.dotify_open_output)
            self.wg_dotify_output_btn.pack(side=Tkinter.RIGHT)
            self.wg_dotify_lbl.pack(side=Tkinter.LEFT)
            self.wg_dotify_output.pack(side=Tkinter.LEFT,fill=Tkinter.X)
            self.command_sel_widgets.append(self.wg_dotify_frame)
            self.command_sel_widgets.append(self.wg_dotify_output)
            self.command_sel_widgets.append(self.wg_dotify_output_btn)
            self.command_sel_widgets.append(self.wg_dotify_lbl)
            self.command_sel_widgets.append(self.wg_rff_check)
            self.set_hint(self.wg_dotify_output,"Choose a target PDF file to which the automata should be written.")

    def dotify_rff_change(self):
        output = self.wg_dotify_output.get()
        if self.rff_check.get():
            if output.endswith(".rmm.pdf"):
                self.wg_dotify_output.delete(0,"end")
                output=output[0:-4]+".rff.pdf"
                self.wg_dotify_output.insert(0,output)
        else:
            if output.endswith(".rff.pdf"):
                self.wg_dotify_output.delete(0,"end")
                output=output[0:-8]+".pdf"
                self.wg_dotify_output.insert(0,output)

    def set_dotify_output_from_code_filename(self):
        try:
            if self.rff_check.get():
                suffix=".rff.pdf"
            else:
                suffix=".pdf"
            self.wg_dotify_output.delete(0,"end")
            if self.get_code_filename() == None:
                self.wg_dotify_output.insert("end","tmp"+suffix)
            else:
                self.wg_dotify_output.insert("end",self.get_code_filename()+suffix)
            self.wg_dotify_output.icursor("end")
        except:
            pass # There is no wg_dotify_output, because a different command is specified

    def dotify_open_output(self):
        filename = tkFileDialog.asksaveasfilename(initialdir=self.conf.last_code_dir,
                                                   title="Save output PDF as",
                                                   filetypes=[("PDF",".pdf")],
                                                   parent=self.master)
        if filename:
            self.wg_dotify_output.delete(0,"end")
            self.wg_dotify_output.insert("end",filename)

    def select_command_from_wg(self):
        self.select_command(self.commands[self.command_sel.get()])

    def abstraction_has_cegar(self,abs):
        return (abs == "pb")

    def select_abstraction_from_wg(self):
        self.fence_check.set(self.abs_sel.get() == "hsb")
        if self.abstraction_has_cegar(self.abs_sel.get()):
            self.wg_cegar_check.config(state=Tkinter.NORMAL)
        else:
            self.wg_cegar_check.config(state=Tkinter.DISABLED)

    def async_output(self,prog,on_done=None):
        self.inc_subthread_count()
        try:
            l = prog.stdout.readline()
            while len(l) > 0:
                if l.startswith("json: "):
                    j = json.loads(l[6:len(l)])
                    if j["action"] == "Syntax Error":
                        self.mark_error_in_code("{0}.{1}".format(j["pos"][0]["lineno"],j["pos"][0]["charno"]))
                    elif j["action"] == "Link Fence":
                        self.link_fence(j["pos"]);
                    else:
                        self.perror("Unknown JSON: "+l)
                        self.wg_output.see("end")
                else:
                    self.poutput(l)
                    self.wg_output.see("end")
                l = prog.stdout.readline()
        except:
            self.perror_clear(str(sys.exc_info()[1])+"\n")
        retval = prog.poll()
        if retval != None and retval != 0:
            if retval == -11:
                self.perror("Segmentation fault\n")
            else:
                self.perror("Process terminated with return code {0}\n".format(retval))
        self.set_running(False)
        if on_done != None:
            on_done()
        self.dec_subthread_count()

    def async_error(self,prog):
        self.inc_subthread_count()
        try:
            l = prog.stderr.readline()
            while len(l) > 0:
                if not(l.startswith("Warning") or l.startswith("warning")):
                    self.perror(l)
                    self.wg_output.see("end")
                else:
                    self.pwarning(l)
                l = prog.stderr.readline()
        except:
            self.perror(str(sys.exc_info()[1])+"\n")
        self.dec_subthread_count()

    def async_update_running_time(self):
        self.inc_subthread_count()
        self.wg_running_lbl.config(text="Running (0 s)")
        start = int(time.time())
        while self.is_running:
            time.sleep(1)
            s = int(time.time()) - start
            self.wg_running_lbl.config(text="Running ({0} s)".format(s))
        self.wg_running_lbl.config(text="")
        self.dec_subthread_count()

    def set_running(self,val):
        if val:
            self.is_running = True
            self.wg_run_btn.config(state=Tkinter.DISABLED)
            self.wg_interrupt_btn.config(state=Tkinter.NORMAL)
            self.wg_running_lbl.config(text="Running (0 s)")
            thread.start_new_thread(self.async_update_running_time,())
        else:
            self.is_running = False
            # self.wg_run_btn.config(state=Tkinter.NORMAL)
            self.wg_interrupt_btn.config(state=Tkinter.DISABLED)
            self.wg_running_lbl.config(text="")

    def opt_interrupt(self):
        if(self.is_running):
            self.interrupt()

    def interrupt(self):
        assert(self.is_running)
        try:
            self.prog.terminate()
        except:
            self.perror("Failed to terminate subprocess.\n")
        self.set_running(False)
        self.perror("Interrupted\n")

    def run_and_output(self,cmd,input=None,on_done=None):
        self.select_output("output")
        self.clear_output()
        self.clear_errors()
        self.set_running(True)
        try:
            self.poutput("$ "+cmd+"\n")
            if input == None:
                prog = subprocess.Popen(cmd.split(),
                                       stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            else:
                prog = subprocess.Popen(cmd.split(),
                                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,stdin=subprocess.PIPE)
                prog.stdin.write(input)
                prog.stdin.close()
            self.prog = prog
            thread.start_new_thread(self.async_output,(prog,on_done))
            thread.start_new_thread(self.async_error,(prog,))
        except:
            self.perror("Failed to start subprocess ({0})\n".format(cmd.split()[0])+str(sys.exc_info()[1])+"\n")
            self.interrupt()

    def opt_run(self):
        if not(self.is_running):
            self.run()

    def inc_subthread_count(self):
        self.subthread_count_lock.acquire()
        self.subthread_count = self.subthread_count + 1
        self.subthread_count_lock.release()

    def dec_subthread_count(self):
        self.subthread_count_lock.acquire()
        self.subthread_count = self.subthread_count - 1
        c = self.subthread_count
        self.subthread_count_lock.release()
        if c == 0:
            self.wg_run_btn.config(state=Tkinter.NORMAL)

    def are_subthreads_exited(self):
        self.subthread_count_lock.acquire()
        e = (self.subthread_count == 0)
        self.subthread_count_lock.release()
        return e

    def run(self):
        if not(self.are_subthreads_exited()):
            # There are still running subthreads, refuse to restart
            return
        self.remove_fence_links()
        if len(self.conf.binary) == 0:
            self.perror_clear("You need to specify the path to the command-line tool. (Misc->Configuration)")
        else:
            if self.verbosity_sel.get() == "Only Results":
                verb=""
            else:
                verb=" -"+"v"*["Only Results","Messages","Debug","Extreme"].index(self.verbosity_sel.get())
            if self.rff_check.get():
                rff=" --rff"
            else:
                rff=""
            if self.fence_check.get():
                fence=" -a hsb"
            else:
                fence=""
            rmm_code = self.wg_code.get("1.0","end")
            if self.commands[self.command_sel.get()] == "reach":
                cmd = self.conf.binary+" reach"+verb+rff+" --json --abstraction "+self.abs_sel.get()
                if self.cegar_check.get() and self.wg_cegar_check["state"] == Tkinter.NORMAL:
                    cmd += " --cegar"
                self.run_and_output(cmd,input=rmm_code)
            elif self.commands[self.command_sel.get()] == "fencins":
                cmd = self.conf.binary+" fencins --json"+verb+rff+" --abstraction "+self.abs_sel.get()
                if self.cegar_check.get() and self.wg_cegar_check["state"] == Tkinter.NORMAL:
                    cmd += " --cegar"
                self.run_and_output(cmd,input=rmm_code)
            elif self.commands[self.command_sel.get()] == "dotify":
                output = self.wg_dotify_output.get()
                cmd = self.conf.binary+" dotify --json"+verb+rff+fence+" -o "+output
                self.run_and_output(cmd,input=rmm_code,on_done=lambda: self.show_dotify_pdf(output))
            else:
                self.perror_clear("Run("+self.commands[self.command_sel.get()]+"): Not implemented")

    def show_dotify_pdf(self,filename):
        if len(self.conf.pdf_viewer)>0:
            try:
                subprocess.Popen([self.conf.pdf_viewer,filename])
                self.poutput(self.conf.pdf_viewer+" "+filename+"\n")
            except:
                self.perror("Failed to show PDF file.\n"+str(sys.exc_info()[1])+"\n")
        else:
            self.poutput("Hint: To automatically open PDF files, specify a PDF viewer in Misc->Configuration.\n")

    def cleanup(self):
        self.conf.write_conf_default()

    def destroy_evt(self,evt):
        if evt.widget == self.master:
            self.cleanup()

    def mark_error_in_code(self,pos):
        tagnum = 0
        while "error_tag{0}".format(tagnum) in self.wg_code.tag_names():
            tagnum = tagnum + 1
        tag = "error_tag{0}".format(tagnum)
        self.wg_code.tag_add(tag,pos)
        self.wg_code.tag_config(tag,background="red")
        self.wg_code.see(pos)

    def clear_error_marks_in_code(self):
        for t in self.wg_code.tag_names():
            if t.startswith("error_tag"):
                self.wg_code.tag_delete(t)
 
    def link_fence(self,pos):
        self.fence_links.append({'pos':pos,
                                 'output_line':len(self.output_text.split('\n'))-1
                                 })
        self.update_fence_link_marks()

    # Remove links from the output widget, but keep remembering the links
    def clear_fence_link_marks_in_output(self):
        for t in self.wg_output.tag_names():
            if t.startswith("fence_link"):
                self.wg_output.tag_delete(t)

    # Remove links from the output widget, and forget them
    def remove_fence_links(self):
        self.clear_fence_link_marks_in_output()
        self.fence_links = []

    # Update the marks in output to point to the right locations in the code
    def update_fence_link_marks(self):
        self.clear_fence_link_marks_in_output()
        if self.output_sel.get() == "output":
            for lnk in self.fence_links:
                c = 0
                for p in lnk['pos']:
                    c = self.update_fence_link_mark(lnk,p,c)
                c = self.update_fence_link_mark(lnk,lnk['pos'][0],c)

    def update_fence_link_mark(self, lnk, pos, c):
        s = self.wg_output.get("{0}.0".format(lnk['output_line']),
                               "{0}.end".format(lnk['output_line']));
        # Avoid spaces at the beginning of the link
        while(s[c] == ' '):
            c = c + 1
        ln = "L{0}".format(pos['lineno'])
        pos0 = "{0}.{1}".format(lnk['output_line'],c)
        try:
            if(s[c:c+len(ln)] == ln):
                pos1 = "{0}.{1}".format(lnk['output_line'],c+len(ln))
            else:
                pos1 = "{0}.end".format(lnk['output_line'])
        except:
            pos1 = "{0}.end".format(lnk['output_line'])
        start = "{0}.{1}".format(pos['lineno'],pos['charno'])
        end = "{0}.{1}".format(pos['lineno'],pos['charno']+5)
        tag = "fence_link{0}".format(len(self.wg_output.tag_names()))
        self.wg_output.tag_add(tag,pos0,pos1)
        self.wg_output.tag_config(tag,foreground="#00aa00",underline=True)
        self.wg_output.tag_bind(tag,"<Enter>",lambda evt, start=start, end=end: self.set_highlight_in_code(start,end))
        self.wg_output.tag_bind(tag,"<Leave>",lambda evt: self.unset_highlights_in_code())
        self.wg_output.tag_bind(tag,"<Button-1>",lambda evt, start=start: self.wg_code.see(start))
        # Increase c to the index of the next linkable item (line number or transition)
        while(c < len(s) and s[c] != ' '):
            c = c + 1;
        if(s[c:c+4] == ' by '):
            c = c+4
        return c

    def unset_highlights_in_code(self):
        self.wg_output.config(cursor="xterm")
        for t in self.wg_code.tag_names():
            if t.startswith("highlight"):
                self.wg_code.tag_delete(t)

    def set_highlight_in_code(self,start,end):
        self.unset_highlights_in_code()
        self.wg_output.config(cursor="hand2")
        num = 0
        while "highlight{0}".format(num) in self.wg_code.tag_names():
            num = num + 1
        tag = "highlight{0}".format(num)
        self.wg_code.tag_add(tag,start,end)
        self.wg_code.tag_config(tag,background="yellow",borderwidth=1,relief=Tkinter.RIDGE)

root = Tkinter.Tk()
root.title("Memorax GUI")

mw = MainWindow(root)

root.geometry("600x800")

root.mainloop()
