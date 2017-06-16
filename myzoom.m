function ha_zoom=myzoom(position,zoomscale,axiscale,ha_zoom)
if ha_zoom~=0
    cla(ha_zoom);
end
ha=get(gcf,'CurrentAxes');
ha_zoom=copyobj(gca,gcf);
set(ha_zoom,'position',position);
set(gcf,'CurrentAxes',ha_zoom)
axis(axiscale)
xlabel('')
ylabel('')
zlabel('')
view(73,40)
zoom(zoomscale);
ha_zoom=get(gcf,'CurrentAxes');
set(gcf,'CurrentAxes',ha)

end