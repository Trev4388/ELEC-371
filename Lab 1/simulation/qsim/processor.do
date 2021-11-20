onerror {quit -f}
vlib work
vlog -work work processor.vo
vlog -work work processor.vt
vsim -novopt -c -t 1ps -L cycloneiii_ver -L altera_ver -L altera_mf_ver -L 220model_ver -L sgate work.processor_vlg_vec_tst
vcd file -direction processor.msim.vcd
vcd add -internal processor_vlg_vec_tst/*
vcd add -internal processor_vlg_vec_tst/i1/*
add wave /*
run -all
