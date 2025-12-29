set terminal pngcairo size 1200,900 enhanced font 'Verdana,10'
set output 'Final_Challenge_Report.png'
set multiplot layout 2,1 title "Real-Time 500Hz Sensor Benchmark (Synced Clocks)"

set title "Signal Integrity (10s Sine Wave)"
set grid
plot "sensor_1.dat" u 1:2 w l title "Sensor 1", "sensor_2.dat" u 1:2 w l title "Sensor 2"

set title "End-to-End Latency Measurement (ms)"
set ylabel "Latency (ms)"
set yrange [0:2]
set grid
plot "sensor_1.dat" u 1:3 w p pt 7 ps 0.4 title "Sensor 1 Latency", \
     "sensor_2.dat" u 1:3 w p pt 7 ps 0.4 title "Sensor 2 Latency"

unset multiplot