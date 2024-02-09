set terminal pngcairo dashed size 3840,2160 font "Helvetica,30"
set output 'simulateEnableLogic.png'
set size 1,1
set origin 0,0
set multiplot layout 3,1
set lmargin at screen 0.05
set style arrow 1 nohead lc rgb 'red' lw 3 dt 2
set style arrow 2 nohead lc rgb 'black' lw 3 dt 2
set format x ""
set arrow 1 arrowstyle 1 from first 87,screen 0.05 to first 87, screen 1.0
set label "U_{Supply} attached"  at first  88, first 12 tc rgb "red"
set arrow 2 arrowstyle 1 from first 150,screen 0.05 to first 150, screen 1.0
set label "U_{Supply} detached"  at first  151, first 7 tc rgb "red"
set arrow 3 arrowstyle 2 from first 227,screen 0.36 to first 227, screen 0.86
set label "U_{Sys} < 17.5V"  at first  207, first 20 tc rgb "black"
set arrow 4 arrowstyle 2 from first 237,screen 0.28 to first 237, screen 0.38
set arrow 5 arrowstyle 1 from first 253,screen 0.35 to first 253, screen 1.0
set label "U_{Supply} attached"  at first  254, first 12 tc rgb "red"
set arrow 6 arrowstyle 1 from first 336,screen 0.05 to first 336, screen 1.0
set label "U_{Supply} detached"  at first  337, first 7 tc rgb "red"
set arrow 7 arrowstyle 2 from first 392,screen 0.36 to first 392, screen 0.86
set label "U_{Sys} < 17.5V"  at first  372, first 20 tc rgb "black"
set arrow 8 arrowstyle 2 from first 403,screen 0.28 to first 403, screen 0.38
set arrow 9 arrowstyle 2 from first 503,screen 0.05 to first 503, screen 0.6
set ylabel "U/V"
p "simulateEnableLogic.txt" u 1:3 w l lw 3 t "U_{sys}", 24.0 w l lw 3 t "U_{thresh high}", 17.5 w l lw 3 t "U_{thresh low}"
unset arrow
unset label
set ylabel "time/s"
set label "t_{U_{low1}}"  at first  215, first -7 tc rgb "black"
set label "t_{U_{low1}+10s}"  at first  239, first -7 tc rgb "black"
set label "t_{U_{low2}}"  at first  381, first -7 tc rgb "black"
set label "t_{U_{low2}+10s}"  at first  405, first -7 tc rgb "black"
set label "t_{U_{low2}+120s}"  at first  506, first -7 tc rgb "black"
set label "enabled"  at first  30, first -80 tc rgb "black"
set label "disabled"  at first  105, first -80 tc rgb "black"
set label "enabled"  at first  180, first -80 tc rgb "black"
set label "disabled"  at first  280, first -80 tc rgb "black"
set label "enabled"  at first  360, first -80 tc rgb "black"
set label "disabled"  at first  440, first -80 tc rgb "black"
set label "System off"  at first  535, first -80 tc rgb "black"
p "simulateEnableLogic.txt" u 1:2 w l lw 3 t "t_{low voltage}", 10 w l lw 3 t "t_{thresh enable}"
unset label
set xlabel "time/s"
set format x "% g"
set yrange [0:1.1]
set ytics (0,1)
set ylabel "enabled"
p "simulateEnableLogic.txt" u 1:4 w l lw 3 t "enabled"
unset multiplot
