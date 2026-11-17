/* Auto Tangents 3.0 | AE 2023 / ExtendScript ES3
 * Custom interpolation, not a reproduction of Cinema 4D's proprietary solver.
 * No expressions, added keys, time/value changes, or spatial path edits.
 */
function AT_monotone(delta, a, b) {
    // Derivative / 3 of cubic Bezier values, oriented towards the endpoint.
    if (delta === 0) return a === 0 && b === 0;
    var sign = delta < 0 ? -1 : 1, d = Math.abs(delta);
    a *= sign; b *= sign;
    if (a < 0 || b < 0) return false;
    var c = 3 * (a + b) - 2 * d, l = 2 * (d - 2 * a - b), u;
    if (c > 0) {
        u = -l / (2 * c);
        if (u > 0 && u < 1 && (c * u + l) * u + a < -1e-12 * Math.max(d, a, b)) return false;
    }
    return true;
}
function AT_removeOvershoot(times, values, incoming, outgoing, inPercent, outPercent, inEditable, outEditable) {
    var ins = incoming.slice(0), outs = outgoing.slice(0), n = times.length;
    var ties = [], i, pass, h, d, sign, a, b, budget, fixed, change, factor, m, adjusted;
    for (i = 1; i < n - 1; i++) ties[i] = inEditable[i] && outEditable[i] && incoming[i] === outgoing[i];
    // Repair only non-monotone segments. Once repaired, their ordered control
    // polygons remain safe under further slope reductions for continuity.
    for (pass = 0; pass <= n; pass++) {
        adjusted = false;
        for (i = 0; i < n - 1; i++) {
            h = times[i + 1] - times[i]; d = values[i + 1] - values[i];
            if (!(h > 0)) throw new Error("Invalid key times");
            a = outs[i] * h * outPercent[i] / 100;
            b = ins[i + 1] * h * inPercent[i + 1] / 100;
            if (AT_monotone(d, a, b)) continue;
            adjusted = true; sign = d < 0 ? -1 : 1;
            if (outEditable[i] && (d === 0 || outs[i] * sign < 0)) outs[i] = 0;
            if (inEditable[i + 1] && (d === 0 || ins[i + 1] * sign < 0)) ins[i + 1] = 0;
            a = Math.abs(outs[i] * h * outPercent[i] / 100);
            b = Math.abs(ins[i + 1] * h * inPercent[i + 1] / 100);
            fixed = (outEditable[i] ? 0 : a) + (inEditable[i + 1] ? 0 : b);
            change = (outEditable[i] ? a : 0) + (inEditable[i + 1] ? b : 0);
            budget = Math.max(0, Math.abs(d) - fixed);
            if (change > budget) {
                factor = budget / change;
                if (outEditable[i]) outs[i] *= factor;
                if (inEditable[i + 1]) ins[i + 1] *= factor;
            }
        }
        for (i = 1; i < n - 1; i++) {
            if (ties[i] && ins[i] !== outs[i]) {
                m = (incoming[i] < 0 ? -1 : 1) * Math.min(Math.abs(ins[i]), Math.abs(outs[i]));
                ins[i] = m; outs[i] = m; adjusted = true;
            }
        }
        if (!adjusted) break;
    }
    return { incoming:ins, outgoing:outs };
}
function AT_weights(times, weighted, strength) {
    var result = [], i, left, right, equal;
    if (!(strength >= 0.1 && strength <= 50)) throw new Error("权重必须在 0.1% 到 50% 之间");
    for (i = 0; i < times.length; i++) {
        left = i > 0 ? times[i] - times[i - 1] : times[1] - times[0];
        right = i < times.length - 1 ? times[i + 1] - times[i] : left;
        if (!(left > 0 && right > 0)) throw new Error("Invalid key times");
        equal = Math.min(left, right) * strength / 100;
        result.push({ left:weighted ? left * strength / 100 : equal,
            right:weighted ? right * strength / 100 : equal,
            inPercent:weighted ? strength : equal / left * 100,
            outPercent:weighted ? strength : equal / right * 100 });
        // AE cannot represent influence below 0.1%; never silently destroy symmetry.
        if (result[i].inPercent < 0.1 - 1e-10 || result[i].outPercent < 0.1 - 1e-10)
            throw new Error("关键帧间距差过大，AE 无法表示等长切线；请启用加权相切");
        result[i].inPercent = Math.max(0.1, result[i].inPercent);
        result[i].outPercent = Math.max(0.1, result[i].outPercent);
    }
    return result;
}
function AT_solve(times, values, safe, fixed, weights) {
    var n = times.length, slopes = [], result = [], i, a, b, h0, h1, m;
    for (i = 0; i < n - 1; i++) {
        if (!(times[i + 1] > times[i])) throw new Error("Invalid key times");
        slopes[i] = (values[i + 1] - values[i]) / (times[i + 1] - times[i]);
    }
    result[0] = 0;
    for (i = 1; i < n - 1; i++) {
        a = slopes[i - 1]; b = slopes[i];
        h0 = times[i] - times[i - 1]; h1 = times[i + 1] - times[i];
        // Neighbor-to-neighbor secant ignores the middle key's time/value.
        // Thus fixed slope stays unchanged when only the middle key moves.
        m = fixed ? (values[i + 1] - values[i - 1]) / (h0 + h1) : (a * h1 + b * h0) / (h0 + h1);
        if (safe) {
            if (a === 0 || b === 0 || (a < 0) !== (b < 0)) m = 0;
            else {
                // Ordered control points guarantee monotonic values.
                // Each control point occupies at most half the value span.
                var limit = weights ? Math.min(Math.abs(values[i] - values[i - 1]) / (2 * weights[i].left),
                    Math.abs(values[i + 1] - values[i]) / (2 * weights[i].right)) : 1.5 * Math.min(Math.abs(a), Math.abs(b));
                m = (a < 0 ? -1 : 1) * Math.min(Math.abs(m), limit);
            }
        }
        if (!isFinite(m)) throw new Error("Non-finite tangent");
        result[i] = m;
    }
    result[n - 1] = 0;
    return result;
}

(function (host) {
    if (typeof module !== "undefined" && module.exports) {
        module.exports = { solve: AT_solve, weights:AT_weights, removeOvershoot:AT_removeOvershoot }; return;
    }
    function vec(v) { return typeof v === "number" ? [v] : v; }
    function snapshot(p, k) {
        return { p:p, k:k, it:p.keyInInterpolationType(k), ot:p.keyOutInterpolationType(k),
            ie:p.keyInTemporalEase(k), oe:p.keyOutTemporalEase(k),
            auto:p.keyTemporalAutoBezier(k), continuous:p.keyTemporalContinuous(k) };
    }
    function restore(s) {
        s.p.setTemporalAutoBezierAtKey(s.k, false);
        s.p.setTemporalContinuousAtKey(s.k, false);
        s.p.setInterpolationTypeAtKey(s.k, s.it, s.ot);
        s.p.setTemporalEaseAtKey(s.k, s.ie, s.oe);
        s.p.setTemporalContinuousAtKey(s.k, s.continuous);
        s.p.setTemporalAutoBezierAtKey(s.k, s.auto);
    }
    function run(mode, safe, whole, weighted, strength, status) {
        if (mode !== "clamp" && (!isFinite(strength) || strength < 0.1 || strength > 50)) {
            alert("权重请输入 0.1 到 50 之间的数字。"); return;
        }
        var comp = app.project.activeItem;
        if (!(comp instanceof CompItem)) { alert("请先打开合成，并选中属性的关键帧。"); return; }
        var props = comp.selectedProperties, plans = [], skipped = [], total = 0;
        var i, j, p, type, keys, runs, current, r, q, times, vals, dim, slopes, s, start, end, weights;
        // Preflight everything before any mutation.
        for (i = 0; i < props.length; i++) {
            p = props[i];
            if (p.propertyType !== PropertyType.PROPERTY || p.numKeys < 2) continue;
            try {
                type = p.propertyValueType;
                if (p.isSpatial) throw new Error("位置/空间属性请先分离维度，再选择 X、Y、Z");
                if (type !== PropertyValueType.OneD && type !== PropertyValueType.TwoD && type !== PropertyValueType.ThreeD)
                    throw new Error("仅支持数值属性；不处理颜色、路径、文本等");
                if (p.expressionEnabled) throw new Error("属性启用了表达式");
                if (!p.isInterpolationTypeValid(KeyframeInterpolationType.BEZIER)) throw new Error("不支持贝塞尔插值");
                keys = []; current = []; runs = [];
                if (whole) { for (j = 1; j <= p.numKeys; j++) keys.push(j); }
                else { keys = p.selectedKeys; }
                if (keys.length < 2) throw new Error("至少选择两个相邻关键帧");
                for (j = 0; j < keys.length; j++) {
                    if (current.length && keys[j] !== current[current.length - 1] + 1) {
                        if (current.length > 1) runs.push(current);
                        current = [];
                    }
                    current.push(keys[j]);
                }
                if (current.length > 1) runs.push(current);
                if (!runs.length) throw new Error("所选关键帧不相邻");
                var propertyPlans = [];
                for (r = 0; r < runs.length; r++) {
                    current = runs[r]; times = []; vals = []; slopes = [];
                    start = current[0]; end = current[current.length - 1];
                    for (j = 0; j < current.length; j++) {
                        q = current[j];
                        if ((q > start && p.keyInInterpolationType(q) === KeyframeInterpolationType.HOLD) ||
                            (q < end && p.keyOutInterpolationType(q) === KeyframeInterpolationType.HOLD))
                            throw new Error("选区内包含保持关键帧，已跳过整个属性");
                        times.push(p.keyTime(q)); vals.push(vec(p.keyValue(q)));
                    }
                    if (mode === "clamp") {
                        var saved = [], repaired = [];
                        for (j = 0; j < current.length; j++) saved.push(snapshot(p, current[j]));
                        for (dim = 0; dim < vals[0].length; dim++) {
                            var cv = [], ci = [], co = [], wi = [], wo = [], mi = [], mo = [];
                            for (j = 0; j < current.length; j++) {
                                s = saved[j]; cv.push(vals[j][dim]);
                                mi.push(s.it === KeyframeInterpolationType.BEZIER);
                                mo.push(s.ot === KeyframeInterpolationType.BEZIER);
                                ci.push(j > 0 && !mi[j] ? (vals[j][dim] - vals[j - 1][dim]) / (times[j] - times[j - 1]) : s.ie[dim].speed);
                                co.push(j < current.length - 1 && !mo[j] ? (vals[j + 1][dim] - vals[j][dim]) / (times[j + 1] - times[j]) : s.oe[dim].speed);
                                wi.push(mi[j] ? s.ie[dim].influence : 100 / 3);
                                wo.push(mo[j] ? s.oe[dim].influence : 100 / 3);
                                if (!isFinite(cv[j]) || !isFinite(ci[j]) || !isFinite(co[j])) throw new Error("关键帧数值无效");
                            }
                            repaired.push(AT_removeOvershoot(times, cv, ci, co, wi, wo, mi, mo));
                        }
                        for (j = 0; j < current.length; j++) {
                            s = saved[j]; var rei = [], reo = [], changed = false;
                            for (dim = 0; dim < repaired.length; dim++) {
                                var ri = j > 0 && s.it === KeyframeInterpolationType.BEZIER ? repaired[dim].incoming[j] : s.ie[dim].speed;
                                var ro = j < current.length - 1 && s.ot === KeyframeInterpolationType.BEZIER ? repaired[dim].outgoing[j] : s.oe[dim].speed;
                                if (ri !== s.ie[dim].speed || ro !== s.oe[dim].speed) changed = true;
                                rei.push(new KeyframeEase(ri, s.ie[dim].influence));
                                reo.push(new KeyframeEase(ro, s.oe[dim].influence));
                            }
                            if (changed) propertyPlans.push({saved:s, ie:rei, oe:reo, it:s.it, ot:s.ot});
                        }
                        continue;
                    }
                    weights = AT_weights(times, mode === "weight" ? true : weighted, strength);
                    for (dim = 0; dim < vals[0].length; dim++) {
                        var v = [];
                        for (j = 0; j < vals.length; j++) {
                            if (!isFinite(vals[j][dim])) throw new Error("关键帧数值无效");
                            v.push(vals[j][dim]);
                        }
                        slopes.push(AT_solve(times, v, safe, mode === "fixed", weights));
                    }
                    for (j = 0; j < current.length; j++) {
                        s = snapshot(p, current[j]);
                        var ei = [], eo = [];
                        for (dim = 0; dim < slopes.length; dim++) {
                            var inSpeed = slopes[dim][j], outSpeed = slopes[dim][j];
                            if (mode === "weight") {
                                // Preserve manual slopes; LINEAR's true slope is its secant,
                                // not the dormant KeyframeEase data returned by AE.
                                inSpeed = j > 0 && s.it === KeyframeInterpolationType.LINEAR ?
                                    (vals[j][dim] - vals[j - 1][dim]) / (times[j] - times[j - 1]) : s.ie[dim].speed;
                                outSpeed = j < current.length - 1 && s.ot === KeyframeInterpolationType.LINEAR ?
                                    (vals[j + 1][dim] - vals[j][dim]) / (times[j + 1] - times[j]) : s.oe[dim].speed;
                            }
                            ei.push(new KeyframeEase(inSpeed, weights[j].inPercent));
                            eo.push(new KeyframeEase(outSpeed, weights[j].outPercent));
                        }
                        propertyPlans.push({ saved:s, ie:j === 0 ? s.ie : ei,
                            oe:j === current.length - 1 ? s.oe : eo,
                            it:j === 0 ? s.it : KeyframeInterpolationType.BEZIER,
                            ot:j === current.length - 1 ? s.ot : KeyframeInterpolationType.BEZIER });
                    }
                }
                for (j = 0; j < propertyPlans.length; j++) plans.push(propertyPlans[j]);
            } catch (e) { skipped.push(p.name + "：" + e.toString()); }
        }
        if (!plans.length) {
            status.text = mode === "clamp" ? "没有需要修正的超调，或没有可处理的选区" : "没有可处理的关键帧";
            if (skipped.length || mode !== "clamp") alert(status.text + "\n" + skipped.join("\n"));
            return;
        }
        app.beginUndoGroup(mode === "clamp" ? "移除超调" : mode === "weight" ? "自动加权（一次性）" : "自动相切");
        var touched = [], failure = null, rollbackErrors = [];
        try {
            for (i = 0; i < plans.length; i++) {
                var plan = plans[i]; s = plan.saved; touched.push(s);
                s.p.setTemporalAutoBezierAtKey(s.k, false);
                s.p.setTemporalContinuousAtKey(s.k, false);
                s.p.setInterpolationTypeAtKey(s.k, plan.it, plan.ot);
                s.p.setTemporalEaseAtKey(s.k, plan.ie, plan.oe);
                total++;
            }
        } catch (err) {
            failure = err;
            for (i = touched.length - 1; i >= 0; i--) {
                try { restore(touched[i]); } catch (re) { rollbackErrors.push(re.toString()); }
            }
        } finally { app.endUndoGroup(); }
        if (failure) {
            status.text = "执行失败";
            alert("执行失败：" + failure.toString() + "\n" +
                (rollbackErrors.length ? "部分恢复失败，请立即 Ctrl+Z 撤销本次操作。" : "已恢复本次涉及的切线设置。"));
            return;
        }
        status.text = (mode === "clamp" ? "移除超调" : mode === "weight" ? "自动加权" : mode === "fixed" ? "固定斜率" : "经典相切") +
            (safe ? "＋防超调" : "") + "：" + total + " 个关键帧；Ctrl+Z 撤销";
        if (skipped.length) alert("部分属性已跳过：\n" + skipped.join("\n"));
    }
    var win = host instanceof Panel ? host : new Window("palette", "自动相切 3.0 · AE 2023", undefined, {resizeable:true});
    win.orientation = "column"; win.alignChildren = ["fill", "top"];
    win.add("statictext", undefined, "先选中至少两个相邻关键帧");
    var autoButton = win.add("button", undefined, "自动相切－经典");
    autoButton.helpTip = "根据相邻关键帧计算平滑切线；选区两端速度为零。此模式允许超调。";
    var fixedButton = win.add("button", undefined, "自动相切－固定斜率");
    fixedButton.helpTip = "内部切线取前后相邻关键帧连线的斜率。仅移动中间关键帧后重新点击，斜率不变。首尾水平。";
    var safeButton = win.add("button", undefined, "移除超调（独立修正）");
    safeButton.helpTip = "修正现有曲线的超调/回摆，保留影响范围及插值类型。不自动相切；不受权重设置影响。";
    var weighted = win.add("checkbox", undefined, "加权相切（仅用于两个自动相切按钮）");
    weighted.value = true;
    weighted.helpTip = "勾选：左右手柄按各自相邻关键帧间距分配长度；取消：左右采用相同时间长度。";
    win.add("statictext", undefined, "切线权重 %（0.1–50；默认 33.333）");
    var strength = win.add("edittext", undefined, "33.33333333333333");
    var weightButton = win.add("button", undefined, "自动加权（一次性近似）");
    weightButton.helpTip = "保留当前斜率，按邻接间距重分配手柄长度。AE 无法锁定横向手柄；不保证消除原有超调。";
    var whole = win.add("checkbox", undefined, "处理所选属性的全部关键帧");
    whole.value = false;
    win.add("statictext", undefined, "位置需先分离维度；修改后重新点击", {multiline:true});
    win.add("statictext", undefined, "自动加权不锁定手柄；非 C4D 精确复刻", {multiline:true});
    var status = win.add("statictext", undefined, "就绪 · 移除超调独立于相切模式", {multiline:true});
    var lastMode = "classic";
    autoButton.onClick = function () { lastMode = "classic"; run(lastMode, false, whole.value, weighted.value, Number(strength.text), status); };
    fixedButton.onClick = function () { lastMode = "fixed"; run(lastMode, false, whole.value, weighted.value, Number(strength.text), status); };
    safeButton.onClick = function () { run("clamp", false, whole.value, weighted.value, Number(strength.text), status); };
    weightButton.onClick = function () { run("weight", false, whole.value, true, Number(strength.text), status); };
    win.onResizing = win.onResize = function () { this.layout.resize(); };
    win.layout.layout(true);
    if (win instanceof Window) { win.center(); win.show(); }
})(this);
