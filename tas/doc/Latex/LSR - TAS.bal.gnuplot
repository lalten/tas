set table "LSR*-*TAS.bal.table"; set format "%.5f"
set format "%.7e";; set xrange [0:10]; f(x)=a*x+b; fit f(x) "data.dat" via a,b; plot f(x)
