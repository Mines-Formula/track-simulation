sMet = load("sMet.mat").s;
sImp = load("sImp.mat").s;
sMet = sMet .* 3.281;
sDiff = sMet - sImp;

dsMet = load("dsMet.mat").ds;
dsImp = load("dsImp.mat").ds;
dsMet = dsMet .* 3.281;
dsDiff = dsMet - dsImp;