function [hbc, hbd, hdf, hef ,hqf ,hqg ,heg ,hmn, hmk, hqk, hqv, hzv, hzk, hed ,hab ,hgm]=ZKDrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, params)
%xzk:改动为传出了一系列句柄
hbc=plot([B(1)/1000,C(1)/1000],[B(2)/1000,C(2)/1000],'k','LineWidth',1);hold on;
hbd=plot([B(1)/1000,D(1)/1000],[B(2)/1000,D(2)/1000],'k','LineWidth',1);
hdf=plot([D(1)/1000,F(1)/1000],[D(2)/1000,F(2)/1000],'k','LineWidth',1);
hef=plot([E(1)/1000,F(1)/1000],[E(2)/1000,F(2)/1000],'k','LineWidth',1);
hqf=plot([Q(1)/1000,F(1)/1000],[Q(2)/1000,F(2)/1000],'k','LineWidth',1);
hqg=plot([Q(1)/1000,G(1)/1000],[Q(2)/1000,G(2)/1000],'k','LineWidth',1);
heg=plot([E(1)/1000,G(1)/1000],[E(2)/1000,G(2)/1000],'k','LineWidth',1);
hmn=plot([M(1)/1000,N(1)/1000],[M(2)/1000,N(2)/1000],'k','LineWidth',1);
hmk=plot([M(1)/1000,K(1)/1000],[M(2)/1000,K(2)/1000],'k','LineWidth',1);
hqk=plot([Q(1)/1000,K(1)/1000],[Q(2)/1000,K(2)/1000],'k','LineWidth',1);
hqv=plot([Q(1)/1000,V(1)/1000],[Q(2)/1000,V(2)/1000],'k','LineWidth',1);
theta = atan2(V(2)-Q(2), V(1)-Q(1));
Z = [V(1) + 2/3*params.QV * cos(pi-params.zeta+theta), V(2) + 2/3*params.QV * sin(pi-params.zeta+theta)];
hzv=plot([Z(1)/1000,V(1)/1000],[Z(2)/1000,V(2)/1000],'k','LineWidth',1);
hzk=plot([Z(1)/1000,K(1)/1000],[Z(2)/1000,K(2)/1000],'k','LineWidth',1);
hed=plot([E(1)/1000,D(1)/1000],[E(2)/1000,D(2)/1000],'g','LineWidth',1);
hab=plot([A(1)/1000,B(1)/1000],[A(2)/1000,B(2)/1000],'g','LineWidth',1);
hgm=plot([G(1)/1000,M(1)/1000],[G(2)/1000,M(2)/1000],'g','LineWidth',1);
end