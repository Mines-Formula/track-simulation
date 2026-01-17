function PlotDXFSplines(dxfFilename)
    % PlotDXFSplines — read DXF SPLINE entities, stitch them end-to-end,
    % then smooth and build one continuous spline in the X–Z plane, and plot.
    %
    % Usage: PlotDXFSplines('FSAE_Track.dxf')

    % ---------- Read entire DXF as lines ----------
    fid = fopen(dxfFilename, 'r');
    if fid < 0
        error('Cannot open DXF file: %s', dxfFilename);
    end
    lines = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace','');
    fclose(fid);
    lines = lines{1};
    nlines = numel(lines);
    trim = @(s) strtrim(s);

    % ---------- Parse SPLINE entities ----------
    splines = {};  % each has .degree, .knots, .ctrlpts(Nx3), .weights
    i = 1;
    while i <= nlines
        if strcmp(trim(lines{i}), 'SPLINE')
            s = struct('degree', [], 'knots', [], 'ctrlpts', [], 'weights', []);
            i = i + 1;
            while i <= nlines
                code = trim(lines{i});
                if strcmp(code, '0')
                    break;
                end
                val = '';
                if i+1 <= nlines, val = trim(lines{i+1}); end

                switch code
                    case '71'
                        s.degree = str2double(val);
                        i = i + 2;
                    case '40'
                        s.knots(end+1) = str2double(val);
                        i = i + 2;
                    case '10'
                        x = str2double(val);
                        y = NaN; z = NaN;
                        if i+3 <= nlines
                            y = str2double(trim(lines{i+2}));
                            z = str2double(trim(lines{i+3}));
                        end
                        s.ctrlpts(end+1, :) = [x, y, z];
                        i = i + 4;
                    case '11'
                        x = str2double(val);
                        y = NaN; z = NaN;
                        if i+3 <= nlines
                            y = str2double(trim(lines{i+2}));
                            z = str2double(trim(lines{i+3}));
                        end
                        s.ctrlpts(end+1, :) = [x, y, z];
                        i = i + 4;
                    case '42'
                        s.weights(end+1,1) = str2double(val);
                        i = i + 2;
                    otherwise
                        i = i + 2;
                end
            end
            if ~isempty(s.ctrlpts) && size(s.ctrlpts,1) >= 2
                splines{end+1} = s;
            end
        else
            i = i + 1;
        end
    end

    if isempty(splines)
        error('No SPLINE entities found in %s.', dxfFilename);
    end

    % ---------- Sample each parsed spline into dense X–Z points ----------
    segmentPts = {};
    for k = 1:numel(splines)
        s = splines{k};
        ctrl_xz = s.ctrlpts(:, [1,3])';  % 2×N
        pts_k = [];
        if ~isempty(s.knots) && numel(s.knots) >= size(ctrl_xz,2) + s.degree + 1
            try
                sp = spmak(s.knots, ctrl_xz);
                uu = linspace(s.knots(1), s.knots(end), 200000);
                pts_k = fnval(sp, uu);
            catch
                % fallback
            end
        end
        if isempty(pts_k)
            uu = linspace(0,1, max(200,5*size(ctrl_xz,2)));
            sp_i = cscvn(ctrl_xz);
            tdom = linspace(sp_i.breaks(1), sp_i.breaks(end), numel(uu));
            pts_k = fnval(sp_i, tdom);
        end
        pts_k = removeDuplicates(pts_k, 1e-6);
        segmentPts{end+1} = pts_k;
    end

    % ---------- Chain segments end-to-end ----------
    used = false(1, numel(segmentPts));
    chain = {};
    [~, seed] = min(cellfun(@(P) P(1,1), segmentPts));
    chain{end+1} = segmentPts{seed};
    used(seed) = true;

    tol = 5e-2;
    while any(~used)
        tail = chain{end};
        tailEnd = tail(:,end).';
        bestIdx = 0; bestFlip = false; bestDist = inf;
        for k = 1:numel(segmentPts)
            if used(k), continue; end
            P = segmentPts{k};
            d1 = norm(tailEnd - P(:,1).');
            d2 = norm(tailEnd - P(:,end).');
            if d1 < bestDist
                bestDist = d1; bestIdx = k; bestFlip = false;
            end
            if d2 < bestDist
                bestDist = d2; bestIdx = k; bestFlip = true;
            end
        end
        if bestIdx == 0 || bestDist > 1.0
            candidates = find(~used);
            [~,kmin] = min(cellfun(@(P) min(norm(tailEnd - P(:,1).'), ...
                                             norm(tailEnd - P(:,end).')), ...
                                     segmentPts(candidates)));
            bestIdx = candidates(kmin);
            if norm(tailEnd - segmentPts{bestIdx}(:,end).') < ...
               norm(tailEnd - segmentPts{bestIdx}(:,1).')
                bestFlip = true;
            else
                bestFlip = false;
            end
        end
        seg = segmentPts{bestIdx};
        if bestFlip, seg = fliplr(seg); end
        if norm(chain{end}(:,end) - seg(:,1)) < tol
            seg = seg(:,2:end);
        end
        chain{end} = [chain{end}, seg];
        used(bestIdx) = true;
    end

    allPts = chain{end};
    if norm(allPts(:,1) - allPts(:,end)) < 0.5
        allPts = allPts(:,1:end-1);
        allPts = [allPts, allPts(:,1)];
    end

    % ---------- Compute approximate arc-length parameter s ----------
    %assume pts are ordered along the track
    diffs = diff(allPts,1,2);
    segLens = sqrt(sum(diffs.^2,1));
    s = [0, cumsum(segLens)];

    % ---------- Clean up duplicates ----------
    % Ensure numeric
    assert(isnumeric(s) && isvector(s), 's must be numeric vector');
    assert(size(allPts,2) == numel(s), 'Mismatch between pts and s');

    % Remove duplicqte s entries
    [su, ia, ic] = unique(s, 'stable');
    if numel(su) < numel(s)
        warning('Removing duplicate s values before smoothing');
        s = su;
        allPts = allPts(:, ia);
    end

    % Remove any NaNs or Inf
    valid = isfinite(s) & isfinite(allPts(1,:)) & isfinite(allPts(2,:));
    if any(~valid)
        warning('Removing NaN/Inf entries before smoothing');
        s = s(valid);
        allPts = allPts(:, valid);
    end

    % ---------- Smoothing with csaps ----------
    p = 0.5;  % smoothing parameter 
    ppx = csaps(s, allPts(1, :), p);
    ppz = csaps(s, allPts(2, :), p);

    % Evaluate smoothed parametric curve
    ss = linspace(s(1), s(end), 200000);
    Xs = fnval(ppx, ss);
    Zs = fnval(ppz, ss);
    smoothedPts = [Xs; Zs];

    % Build final spline from smoothed data
    trackSplineXZ = cscvn(smoothedPts);

    % Sample for plotting
    tt = linspace(trackSplineXZ.breaks(1), trackSplineXZ.breaks(end), 200000);
    fitPts = fnval(trackSplineXZ, tt);

    % ---------- Plot ----------
    figure; hold on; axis equal;
    title(sprintf('Smoothed stitched track from %s', dxfFilename));
    xlabel('X'); ylabel('Z');

    for k = 1:numel(segmentPts)
        P = segmentPts{k};
        plot(P(1,:), P(2,:), '-', 'Color',[0.8 0.8 0.8]);
    end
    plot(allPts(1,:), allPts(2,:), 'r-', 'LineWidth', 1);
    plot(fitPts(1,:), fitPts(2,:), 'k-', 'LineWidth', 2);

    hold off;

    % Assign into workspace & save
    assignin('base','trackSplineXZ', trackSplineXZ);
    save('TrackSpline.mat', 'trackSplineXZ');
end

function Q = removeDuplicates(P, tol)
    keep = true(1, size(P,2));
    for j = 2:size(P,2)
        if norm(P(:,j) - P(:,j-1)) < tol
            keep(j) = false;
        end
    end
    Q = P(:, keep);
end





PlotDXFSplines('Track.dxf')
