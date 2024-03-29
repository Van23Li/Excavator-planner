function Z = DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, paramets, visual)
if visual == 1
    %%
    plot([B(1)/1000,C(1)/1000],[B(2)/1000,C(2)/1000],'k','LineWidth',1);
    hold on
    plot([B(1)/1000,D(1)/1000],[B(2)/1000,D(2)/1000],'k','LineWidth',1);
    hold on
    plot([D(1)/1000,F(1)/1000],[D(2)/1000,F(2)/1000],'k','LineWidth',1);
    hold on
    plot([E(1)/1000,F(1)/1000],[E(2)/1000,F(2)/1000],'k','LineWidth',1);
    hold on
    plot([Q(1)/1000,F(1)/1000],[Q(2)/1000,F(2)/1000],'k','LineWidth',1);
    hold on
    plot([Q(1)/1000,G(1)/1000],[Q(2)/1000,G(2)/1000],'k','LineWidth',1);
    hold on
    plot([E(1)/1000,G(1)/1000],[E(2)/1000,G(2)/1000],'k','LineWidth',1);
    hold on
    plot([M(1)/1000,N(1)/1000],[M(2)/1000,N(2)/1000],'k','LineWidth',1);
    hold on
    plot([M(1)/1000,K(1)/1000],[M(2)/1000,K(2)/1000],'k','LineWidth',1);
    hold on
    plot([Q(1)/1000,K(1)/1000],[Q(2)/1000,K(2)/1000],'k','LineWidth',1);
    hold on
    plot([Q(1)/1000,V(1)/1000],[Q(2)/1000,V(2)/1000],'k','LineWidth',1);
    hold on
    
    theta = atan2(V(2)-Q(2), V(1)-Q(1));
    Z = [V(1) + 2/3*paramets.QV * cos(pi-paramets.zeta+theta), V(2) + 2/3*paramets.QV * sin(pi-paramets.zeta+theta)];
    plot([Z(1)/1000,V(1)/1000],[Z(2)/1000,V(2)/1000],'k','LineWidth',1);
    hold on
    plot([Z(1)/1000,K(1)/1000],[Z(2)/1000,K(2)/1000],'k','LineWidth',1);
    hold on
    
    plot([E(1)/1000,D(1)/1000],[E(2)/1000,D(2)/1000],'g','LineWidth',1);
    hold on
    plot([A(1)/1000,B(1)/1000],[A(2)/1000,B(2)/1000],'g','LineWidth',1);
    hold on
    plot([G(1)/1000,M(1)/1000],[G(2)/1000,M(2)/1000],'g','LineWidth',1);
    hold on
else
    theta = atan2(V(2)-Q(2), V(1)-Q(1));
    Z = [V(1) + 2/3*paramets.QV * cos(pi-paramets.zeta+theta), V(2) + 2/3*paramets.QV * sin(pi-paramets.zeta+theta)];
end

end