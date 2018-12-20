set term pdf

set xtics 0,200
set mxtics 2
set xrange [-1.1:]
set xlabel "gen"
set ylabel "score"

set output "genetic.pdf"
plot "run_0.genetic.out" u 1:2 w p ps 0.5 pt 6 lc "black" noti, "< fgrep BEST run_0.genetic.out | sed 's/# //'" u 2:3 w lp ps 0.5 pt 5 lc "red" noti, \
     "run_1.genetic.out" u ($1+200):2 w p ps 0.5 pt 6 lc "blue" noti, "< fgrep BEST run_1.genetic.out | sed 's/# //'" u ($2+200):3 w lp ps 0.5 pt 5 lc "red" noti, \
     "run_2.genetic.out" u ($1+400):2 w p ps 0.5 pt 6 lc "green" noti, "< fgrep BEST run_2.genetic.out | sed 's/# //'" u ($2+400):3 w lp ps 0.5 pt 5 lc "red" noti, \
     "run_3.genetic.out" u ($1+600):2 w p ps 0.5 pt 6 lc "cyan" noti, "< fgrep BEST run_3.genetic.out | sed 's/# //'" u ($2+600):3 w lp ps 0.5 pt 5 lc "red" noti, \
     "run_4.genetic.out" u ($1+1600):2 w p ps 0.5 pt 6 lc "magenta" noti, "< fgrep BEST run_4.genetic.out | sed 's/# //'" u ($2+1600):3 w lp ps 0.5 pt 5 lc "red" noti
