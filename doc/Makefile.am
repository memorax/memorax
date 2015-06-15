nobase_dist_doc_DATA = \
   manual.pdf \
   examples/bakery.bound2.rmm \
   examples/bakery.rmm \
   examples/burns.rmm \
   examples/clh.rmm \
   examples/dekker.2.rmm \
   examples/dekker.rmm \
   examples/dijkstra.rmm \
   examples/lamport_fast.rmm \
   examples/peterson.rmm \
   examples/sense_rev_bar.rmm \
   examples/ticket_spin_lock.2.rmm \
   examples/ticket_spin_lock.rmm

EXTRA_DIST = manual.tex biblio.bib fdl-1.3.tex fdl.pdf Makefile.am

fdl.pdf: @FDLDEP@
	$(AM_V_GEN)@FDLBUILDRULE@

manual.pdf: @MANUALDEP@
	$(AM_V_GEN)@MANUALBUILDRULE@

clean-local:
	@MANUALCLEANRULE@ bakery.bound2.rmm

