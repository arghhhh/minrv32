all: complete cover honest imemcheck dmemcheck 


clean:
	rm -r complete cover honest imemcheck dmemcheck checks

checks/makefile:
	python3 genchecks.py
.PHONY: checks/makefile

checks: checks/makefile
	make -C checks -j$(nproc)
.PHONY: checks

complete: complete/status
complete/status:
	sby -f complete.sby
.PHONY: complete

cover: cover/status
cover/status:
	sby -f cover.sby
.PHONY: cover

honest: honest/status
honest/status:
	sby -f honest.sby
.PHONY: honest

imemcheck: imemcheck/status
imemcheck/status:
	sby -f imemcheck.sby
.PHONY: imemcheck

dmemcheck: dmemcheck/status
dmemcheck/status:
	sby -f dmemcheck.sby
.PHONY: dmemcheck

