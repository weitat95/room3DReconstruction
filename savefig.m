function savefig(figure, filename)
    f1 = figure;
    set(f1, 'Units', 'Inches');
    pos1 = get(f1, 'Position');
    set(f1, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos1(3),pos1(4)])
    print(f1,'-dpng','-r0',filename);
end

