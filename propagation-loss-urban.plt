set terminal png
set output "propagation-loss-urban.png"
set title "Urban Propagation Loss vs Path"

set xlabel 'X Pos (m)' offset 0,0,-5
set ylabel 'Y Pos (m)' offset 0,0,0
set zlabel 'Path loss (dB)' offset 0,0,0
set xrange [0:+900]
set yrange [0:+960]
set zrange [0:+200]
set ticslevel 0
set boxwidth 0.9 relative
set style fill pattern
splot "-"  with boxes title "SUI"
80 0 112.035
80 0 112.035
100 0 116.099
150 0 123.891
200 0 129.609
250 0 134.104
300 0 137.802
350 0 140.941
400 0 143.666
450 0 146.074
500 0 148.231
550 0 150.183
600 0 151.966
650 0 153.608
700 0 155.128
750 0 156.544
790 0 157.61
790 50 157.651
790 100 157.774
e
