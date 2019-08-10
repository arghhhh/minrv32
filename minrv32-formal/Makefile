all: complete cover honest imemcheck dmemcheck checks


clean:
	rm -r complete cover honest imemcheck dmemcheck checks

checks:
	python3 genchecks.py
	make -C checks -j$(nproc)


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

