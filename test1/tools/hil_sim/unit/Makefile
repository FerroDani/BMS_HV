# Host unit tests of Core/Src/bms_monitor.c (gcc, no hardware needed)
CFLAGS = -std=c11 -O1 -Wall -Wextra -Werror -DBMS_ENABLE_LOG=0 -Istub -I../../../Core/Inc
SRC    = test_monitor.c ../../../Core/Src/bms_monitor.c

test: $(SRC)
	@for n in 1 11 12; do $(CC) $(CFLAGS) -DBMS_N_SLAVES=$${n}U -o test_monitor_$$n $(SRC) && ./test_monitor_$$n || exit 1; done

clean:
	rm -f test_monitor_*
