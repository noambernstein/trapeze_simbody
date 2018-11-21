set term pdf
set output "genetic.pdf"

set xtics 0,10
set xrange [-0.1:]
set xlabel "gen"
set ylabel "score"
plot "genetic.out" u 1:2 w p pt 6 noti, "< fgrep BEST genetic.out | sed 's/# //'" u 2:3 w lp pt 5 lc "red" noti
bgen=`fgrep BEST genetic.out | tail -1 | awk '{print $3}'`
b=`fgrep BEST genetic.out | tail -1 | awk '{print $4}'`
print "best ", bgen, " ", b
