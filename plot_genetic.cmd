set term pdf

set xtics 0,50
set xrange [-1.1:]
set xlabel "gen"
set ylabel "score"

set output "genetic.pdf"
plot "run_0.genetic.out" u 1:2 w p pt 6 noti, "< fgrep BEST run_0.genetic.out | sed 's/# //'" u 2:3 w lp pt 5 lc "red" noti, \
     "run_1.genetic.out" u ($1+200):2 w p pt 6 noti, "< fgrep BEST run_1.genetic.out | sed 's/# //'" u ($2+200):3 w lp pt 5 lc "red" noti, \
     "run_2.genetic.out" u ($1+400):2 w p pt 6 noti, "< fgrep BEST run_2.genetic.out | sed 's/# //'" u ($2+400):3 w lp pt 5 lc "red" noti
