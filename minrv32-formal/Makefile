

MINRV_SRCS:= ../minlib.v ../minrv32.v


# insn runs all the individual instruction checks in the checks directory
# consistency runs the non-insn checks in the checks directory

all: complete cover insn consistency report

# these are to do with memory...
# honest imemcheck dmemcheck 


clean:
	rm -r complete cover honest imemcheck dmemcheck checks

checks/makefile: $(MINRV_SRCS)
	python3 genchecks.py
.PHONY: checks/makefile

checks: checks/makefile $(MINRV_SRCS)
	make -C checks -j$(nproc)
.PHONY: checks

insn: $(MINRV_SRCS) checks
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

report:
	find . -name PASS | wc
	@find . -name FAIL
