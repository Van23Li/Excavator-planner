function Z = DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, paramets)
%输入：挖掘机各关键点在世界坐标系下的坐标
%绘制挖掘机工作装置
%%
plot([B(1)/1000,C(1)/1000],[B(2)/1000,C(2)/1000],'r','LineWidth',1);
hold on
plot([B(1)/1000,D(1)/1000],[B(2)/1000,D(2)/1000],'r','LineWidth',1);
hold on
plot([D(1)/1000,F(1)/1000],[D(2)/1000,F(2)/1000],'r','LineWidth',1);
hold on
plot([E(1)/1000,F(1)/1000],[E(2)/1000,F(2)/1000],'r','LineWidth',1);
hold on
plot([Q(1)/1000,F(1)/1000],[Q(2)/1000,F(2)/1000],'r','LineWidth',1);
hold on
plot([Q(1)/1000,G(1)/1000],[Q(2)/1000,G(2)/1000],'r','LineWidth',1);
hold on
plot([E(1)/1000,G(1)/1000],[E(2)/1000,G(2)/1000],'r','LineWidth',1);
hold on
plot([M(1)/1000,N(1)/1000],[M(2)/1000,N(2)/1000],'r','LineWidth',1);
hold on
plot([M(1)/1000,K(1)/1000],[M(2)/1000,K(2)/1000],'r','LineWidth',1);
hold on
plot([Q(1)/1000,K(1)/1000],[Q(2)/1000,K(2)/1000],'r','LineWidth',1);
hold on
plot([Q(1)/1000,V(1)/1000],[Q(2)/1000,V(2)/1000],'r','LineWidth',1);
hold on

theta = atan2(V(2)-Q(2), V(1)-Q(1));
Z = [V(1) + 2/3*paramets.QV * cos(pi-paramets.zeta+theta), V(2) + 2/3*paramets.QV * sin(pi-paramets.zeta+theta)];
plot([Z(1)/1000,V(1)/1000],[Z(2)/1000,V(2)/1000],'r','LineWidth',1);
hold on
plot([Z(1)/1000,K(1)/1000],[Z(2)/1000,K(2)/1000],'r','LineWidth',1);
hold on

plot([E(1)/1000,D(1)/1000],[E(2)/1000,D(2)/1000],'y','LineWidth',1);
hold on
plot([A(1)/1000,B(1)/1000],[A(2)/1000,B(2)/1000],'y','LineWidth',1);
hold on
plot([G(1)/1000,M(1)/1000],[G(2)/1000,M(2)/1000],'y','LineWidth',1);
hold on

end