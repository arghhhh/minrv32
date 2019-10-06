

MINRV_SRCS:= ../minrv32.v



all: complete cover honest imemcheck dmemcheck insn consistency


clean:
	rm -r complete cover honest imemcheck dmemcheck checks

checks/makefile: $(MINRV_SRCS)
	python3 genchecks.py
.PHONY: checks/makefile

checks: checks/makefile $(MINRV_SRCS)
	make -C checks -j$(nproc)
.PHONY: checks

insn: $(MINRV_SRCS)
	make -C checks -j$(nproc) insn

consistency: $(MINRV_SRCS)
	make -C checks consistency



complete: complete/status
complete/status: $(MINRV_SRCS)
	sby -f complete.sby
.PHONY: complete

cover: cover/status
cover/status: $(MINRV_SRCS)
	sby -f cover.sby
.PHONY: cover

honest: honest/status
honest/status: $(MINRV_SRCS)
	sby -f honest.sby
.PHONY: honest

imemcheck: imemcheck/status
imemcheck/status: $(MINRV_SRCS)
	sby -f imemcheck.sby
.PHONY: imemcheck

dmemcheck: dmemcheck/status
dmemcheck/status: $(MINRV_SRCS)
	sby -f dmemcheck.sby
.PHONY: dmemcheck

