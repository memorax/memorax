AUTOMAKE_OPTIONS = -Wno-override ## Make autotools quit complaining about explicit rule for gui

dist_EXTRA = Makefile.am
EXTRA_PROGRAMS = memorax-gui
bin_PROGRAMS = memorax @GUI@
memorax_SOURCES = ap_list.tcc ap_list.h \
automaton.cpp automaton.h \
cegar_reachability.cpp cegar_reachability.h \
channel_bwd.h channel_bwd.cpp \
channel_constraint.h channel_constraint.cpp \
channel_container.cpp channel_container.h \
cmsat.h \
constraint_container1.cpp constraint_container1.h \
constraint_container.h \
constraint.h \
exact_bwd.cpp exact_bwd.h \
fence_sync.h fence_sync.cpp \
fencins.h fencins.cpp \
intersection_iterator.h \
lang.tcc lang.h \
lexer.cpp lexer.h \
log.cpp log.h \
machine.cpp machine.h \
main.cpp \
min_coverage.cpp min_coverage.h min_coverage.tcc \
parser.cpp parser.h \
pb_cegar.cpp pb_cegar.h \
pb_constraint.cpp pb_constraint.h \
pb_container1.cpp pb_container1.h \
pb_container2.cpp pb_container2.h \
predicates.tcc predicates.h \
preprocessor.h preprocessor.cpp \
pputils.h \
pso_fencins.h pso_fencins.cpp \
pso_slock_sync.h pso_slock_sync.cpp \
hsb_constraint.h hsb_constraint.cpp hsb_constraint.test.cpp \
hsb_container.h hsb_container.cpp \
hsb_pso_bwd.cpp hsb_pso_bwd.h \
reachability.cpp reachability.h \
sb_constraint.cpp sb_constraint.h \
sb_tso_bwd.cpp sb_tso_bwd.h \
shared.h \
sharinglist.tcc sharinglist.h \
shellcmd.cpp shellcmd.h \
sync.h sync.cpp \
sync_set_printer.h sync_set_printer.cpp \
syntax_string.tcc syntax_string.h \
test.h test.cpp \
test_vips_fencins.h test_vips_fencins.cpp \
ticket_queue.h \
timer.h timer.cpp \
trace.cpp trace.h \
trace_fencer.h \
tso_cycle.cpp tso_cycle.h \
tso_cycle_lock.cpp tso_cycle_lock.h \
tso_fence_sync.h tso_fence_sync.cpp \
tso_fencins.cpp tso_fencins.h \
tso_lock_sync.h tso_lock_sync.cpp \
tso_simple_fencer.h tso_simple_fencer.cpp \
tso_var.cpp tso_var.h \
vecset.h vecset.tcc \
vips_bit_constraint.h vips_bit_constraint.cpp \
vips_bit_reachability.h vips_bit_reachability.cpp \
vips_fence_sync.h vips_fence_sync.cpp \
vips_simple_fencer.h vips_simple_fencer.cpp \
vips_syncrd_sync.h vips_syncrd_sync.cpp \
vips_syncwr_sync.h vips_syncwr_sync.cpp \
vqueue.h vqueue.tcc \
zstar.h zstar.tcc
memorax_gui_SOURCES = gui.py

memorax-gui$(EXEEXT): $(srcdir)/gui.py
	$(AM_V_GEN)cat $(srcdir)/gui.py | sed 's#%%PYTHON%%#$(PYTHON)#g' | \
	sed 's#%%BINARY%%#$(bindir)/memorax#g' | \
	sed 's#%%GUI_STRING%%#@PACKAGE_NAME@ GUI @PACKAGE_VERSION@#g' > memorax-gui && \
	chmod a+x memorax-gui
