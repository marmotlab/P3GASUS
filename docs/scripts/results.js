(function () {
  const colors = ["#21a366", "#2189cf", "#d67b31", "#8b5cf6", "#d64f4f", "#586069"];

  const datasets = [
    {
      id: "discrete-python",
      label: "Discrete Python",
      path: "assets/P3GASUS Final Results - Discrete Python.csv",
      parser: parseHeaderCsv,
      xKey: "NUM_AGENTS",
      xLabel: "Agents",
      metrics: [
        {
          id: "time",
          label: "Graph Creation Time",
          unit: "s",
          series: [
            ["ADG", "ADG_Time"],
            ["SAGE", "SAGE_TIME"],
            ["FORTED", "FORTED_TIME"],
            ["MAGE", "MAGE_TIME"]
          ]
        },
        {
          id: "comms",
          label: "Communication Edges",
          unit: "edges",
          series: [
            ["ADG", "ADG_Comms"],
            ["SAGE", "SAGE_Comms"],
            ["FORTED", "FORTED_Comms"],
            ["MAGE", "MAGE_Comms"]
          ]
        }
      ]
    },
    {
      id: "discrete-cpp",
      label: "Discrete C++",
      path: "assets/discrete_cpp.csv",
      parser: parseHeaderCsv,
      xKey: "NUM_AGENTS",
      xLabel: "Agents",
      metrics: [
        {
          id: "time",
          label: "Graph Creation Time",
          unit: "s",
          series: [
            ["ADG", "CPP_OriginalADG_Time"],
            ["SAGE", "CPP_SAGE_Time"],
            ["FORTED", "CPP_FORTED_Time"],
            ["MAGE + FORTED", "CPP_MAGE_FORTED_Time"],
            ["MAGE + SAGE", "CPP_MAGE_SAGE_Time"]
          ]
        },
        {
          id: "comms",
          label: "Communication Edges",
          unit: "edges",
          series: [
            ["ADG", "CPP_OriginalADG_Comms"],
            ["SAGE", "CPP_SAGE_Comms"],
            ["FORTED", "CPP_FORTED_Comms"],
            ["MAGE + FORTED", "CPP_MAGE_FORTED_Comms"],
            ["MAGE + SAGE", "CPP_MAGE_SAGE_Comms"]
          ]
        }
      ]
    },
    {
      id: "continuous-python",
      label: "Continuous Python",
      path: "assets/P3GASUS Final Results - Continuous Python.csv",
      parser: parseContinuousCsv,
      xKey: "agents",
      xLabel: "Agents",
      metrics: [
        {
          id: "sage-speedup",
          label: "SAGE Speedup vs ADG",
          unit: "x",
          groupKey: "frequencyHz",
          seriesFromGroups: true,
          valueKey: "sageSpeedup"
        },
        {
          id: "mage-speedup",
          label: "MAGE Speedup vs ADG",
          unit: "x",
          groupKey: "frequencyHz",
          seriesFromGroups: true,
          valueKey: "mageSpeedup"
        },
        {
          id: "comms-ratio",
          label: "MAGE Communication Reduction",
          unit: "x",
          groupKey: "frequencyHz",
          seriesFromGroups: true,
          valueKey: "mageCommsRatio"
        }
      ]
    }
  ];

  const state = {
    dataById: new Map(),
    dataset: datasets[0],
    metric: datasets[0].metrics[0],
    scale: "linear",
    selectedSeries: new Set(),
    colorBySeries: new Map(),
    lastChart: null,
    animationFrame: null
  };

  const datasetSelect = document.getElementById("dataset-select");
  const metricSelect = document.getElementById("metric-select");
  const scaleSelect = document.getElementById("scale-select");
  const methodCheckboxes = document.getElementById("method-checkboxes");
  const svg = document.getElementById("results-chart");
  const legend = document.getElementById("chart-legend");
  const caption = document.getElementById("chart-caption");
  const tooltip = document.getElementById("chart-tooltip");
  const table = document.getElementById("summary-table");

  if (!datasetSelect || !metricSelect || !svg) return;

  init();

  async function init() {
    datasetSelect.innerHTML = datasets
      .map((dataset) => `<option value="${dataset.id}">${dataset.label}</option>`)
      .join("");
    datasetSelect.addEventListener("change", onDatasetChange);
    metricSelect.addEventListener("change", onMetricChange);
    scaleSelect.addEventListener("change", onScaleChange);
    window.addEventListener("resize", () => render());

    await Promise.all(datasets.map(loadDataset));
    populateMetrics();
    render();
  }

  async function loadDataset(dataset) {
    try {
      const response = await fetch(dataset.path);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const text = await response.text();
      state.dataById.set(dataset.id, dataset.parser(text));
    } catch (error) {
      state.dataById.set(dataset.id, []);
      console.warn(`Could not load ${dataset.path}`, error);
    }
  }

  function onDatasetChange() {
    state.dataset = datasets.find((dataset) => dataset.id === datasetSelect.value) || datasets[0];
    populateMetrics();
    populateSeriesControls();
    render();
  }

  function onMetricChange() {
    state.metric = state.dataset.metrics.find((metric) => metric.id === metricSelect.value) || state.dataset.metrics[0];
    populateSeriesControls();
    render();
  }

  function onScaleChange() {
    state.scale = scaleSelect.value;
    render();
  }

  function populateMetrics() {
    metricSelect.innerHTML = state.dataset.metrics
      .map((metric) => `<option value="${metric.id}">${metric.label}</option>`)
      .join("");
    state.metric = state.dataset.metrics[0];
    populateSeriesControls();
  }

  function populateSeriesControls() {
    const rows = state.dataById.get(state.dataset.id) || [];
    const availableSeries = buildSeries(rows, state.dataset, state.metric);
    state.colorBySeries = buildColorMap(availableSeries);
    const previous = state.selectedSeries;
    const previousStillAvailable = availableSeries
      .map((item) => item.label)
      .filter((label) => previous.has(label));

    const sameMetric = previousStillAvailable.length === availableSeries.length;
    state.selectedSeries = new Set(
      sameMetric ? previousStillAvailable : availableSeries.map((item) => item.label)
    );

    methodCheckboxes.innerHTML = availableSeries
      .map((item, index) => {
        const checked = state.selectedSeries.has(item.label) ? "checked" : "";
        const color = colorForSeries(item.label, index);
        return `
          <label>
            <input type="checkbox" value="${escapeAttribute(item.label)}" ${checked}>
            <span class="legend-swatch" style="background:${color}"></span>
            ${item.label}
          </label>
        `;
      })
      .join("");

    methodCheckboxes.querySelectorAll("input").forEach((checkbox) => {
      checkbox.addEventListener("change", onSeriesToggle);
    });
  }

  function render() {
    cancelChartAnimation();
    const rows = state.dataById.get(state.dataset.id) || [];
    const allSeries = buildSeries(rows, state.dataset, state.metric);
    let series = allSeries.filter((item) => state.selectedSeries.has(item.label));
    if (!series.length && allSeries.length) {
      state.selectedSeries.add(allSeries[0].label);
      const firstCheckbox = methodCheckboxes.querySelector(`input[value="${cssEscape(allSeries[0].label)}"]`);
      if (firstCheckbox) firstCheckbox.checked = true;
      series = [allSeries[0]];
    }
    const points = series.flatMap((item) => item.points);

    if (!rows.length || !points.length) {
      cancelChartAnimation();
      svg.innerHTML = "";
      caption.textContent = "Unable to load CSV data. If you opened this page directly as a file, serve the docs folder locally so browser fetch can read the assets.";
      legend.innerHTML = "";
      table.innerHTML = "";
      return;
    }

    const chart = makeChart(series, points);
    drawChart(chart);
    state.lastChart = chart;
    renderLegend(series);
    renderTable(series);
    caption.textContent = `${state.dataset.label}: ${state.metric.label}. Use the controls to switch dataset, metric, and scale.`;
  }

  function makeChart(series, points) {
    const width = Math.max(svg.clientWidth || 900, 640);
    const height = 420;
    const margin = { top: 22, right: 28, bottom: 54, left: 72 };
    const innerWidth = width - margin.left - margin.right;
    const innerHeight = height - margin.top - margin.bottom;

    const xValues = points.map((point) => point.x);
    const yValues = points.map((point) => point.y).filter((value) => Number.isFinite(value) && value > 0);
    const xMin = Math.min(...xValues);
    const xMax = Math.max(...xValues);
    const rawYMin = state.scale === "log" ? Math.min(...yValues) : 0;
    const rawYMax = Math.max(...yValues);
    const yMin = state.scale === "log" ? Math.max(rawYMin * 0.85, Number.MIN_VALUE) : 0;
    const yMax = rawYMax * 1.08;

    const xScale = (x) => margin.left + ((x - xMin) / Math.max(xMax - xMin, 1)) * innerWidth;
    const yScale = (y) => {
      if (state.scale === "log") {
        const logMin = Math.log10(yMin);
        const logMax = Math.log10(yMax);
        return margin.top + (1 - (Math.log10(Math.max(y, yMin)) - logMin) / Math.max(logMax - logMin, 1)) * innerHeight;
      }
      return margin.top + (1 - (y - yMin) / Math.max(yMax - yMin, 1)) * innerHeight;
    };

    return {
      width,
      height,
      margin,
      innerWidth,
      innerHeight,
      xMin,
      xMax,
      yMin,
      yMax,
      xScale,
      yScale,
      series,
      key: `${state.dataset.id}:${state.metric.id}:${state.scale}`
    };
  }

  function drawChart(chart) {
    svg.setAttribute("viewBox", `0 0 ${chart.width} ${chart.height}`);
    svg.innerHTML = "";
    drawGrid(
      svg,
      chart.width,
      chart.height,
      chart.margin,
      chart.innerWidth,
      chart.innerHeight,
      chart.xMin,
      chart.xMax,
      chart.yMin,
      chart.yMax,
      chart.xScale,
      chart.yScale
    );
    drawSeries(svg, chart.series, chart.xScale, chart.yScale);
  }

  function onSeriesToggle(event) {
    const checkbox = event.currentTarget;
    if (checkbox.checked) {
      state.selectedSeries.add(checkbox.value);
    } else {
      state.selectedSeries.delete(checkbox.value);
    }
    renderWithYAxisTransition();
  }

  function renderWithYAxisTransition() {
    const rows = state.dataById.get(state.dataset.id) || [];
    const allSeries = buildSeries(rows, state.dataset, state.metric);
    let series = allSeries.filter((item) => state.selectedSeries.has(item.label));
    if (!series.length && allSeries.length) {
      state.selectedSeries.add(allSeries[0].label);
      const firstCheckbox = methodCheckboxes.querySelector(`input[value="${cssEscape(allSeries[0].label)}"]`);
      if (firstCheckbox) firstCheckbox.checked = true;
      series = [allSeries[0]];
    }

    const points = series.flatMap((item) => item.points);
    if (!rows.length || !points.length) {
      render();
      return;
    }

    const nextChart = makeChart(series, points);
    const previousChart = state.lastChart;
    renderLegend(series);
    renderTable(series);
    caption.textContent = `${state.dataset.label}: ${state.metric.label}. Use the controls to switch dataset, metric, and scale.`;

    if (!canAnimateYAxis(previousChart, nextChart)) {
      cancelChartAnimation();
      drawChart(nextChart);
      state.lastChart = nextChart;
      return;
    }

    cancelChartAnimation();
    const duration = 320;
    const start = performance.now();

    const step = (now) => {
      const progress = Math.min((now - start) / duration, 1);
      const eased = 1 - Math.pow(1 - progress, 3);
      const yMin = interpolateDomain(previousChart.yMin, nextChart.yMin, eased);
      const yMax = interpolateDomain(previousChart.yMax, nextChart.yMax, eased);
      const animatedChart = withYDomain(nextChart, yMin, yMax);
      drawChart(animatedChart);
      state.lastChart = animatedChart;

      if (progress < 1) {
        state.animationFrame = requestAnimationFrame(step);
      } else {
        state.animationFrame = null;
        state.lastChart = nextChart;
      }
    };

    state.animationFrame = requestAnimationFrame(step);
  }

  function canAnimateYAxis(previousChart, nextChart) {
    return (
      previousChart &&
      previousChart.key === nextChart.key &&
      previousChart.width === nextChart.width &&
      previousChart.height === nextChart.height &&
      previousChart.xMin === nextChart.xMin &&
      previousChart.xMax === nextChart.xMax
    );
  }

  function withYDomain(chart, yMin, yMax) {
    const yScale = (y) => {
      if (state.scale === "log") {
        const logMin = Math.log10(yMin);
        const logMax = Math.log10(yMax);
        return chart.margin.top + (1 - (Math.log10(Math.max(y, yMin)) - logMin) / Math.max(logMax - logMin, 1)) * chart.innerHeight;
      }
      return chart.margin.top + (1 - (y - yMin) / Math.max(yMax - yMin, 1)) * chart.innerHeight;
    };

    return {
      ...chart,
      yMin,
      yMax,
      yScale
    };
  }

  function interpolateDomain(from, to, progress) {
    if (state.scale === "log") {
      return 10 ** (Math.log10(from) + (Math.log10(to) - Math.log10(from)) * progress);
    }
    return from + (to - from) * progress;
  }

  function cancelChartAnimation() {
    if (state.animationFrame) {
      cancelAnimationFrame(state.animationFrame);
      state.animationFrame = null;
    }
  }

  function drawGrid(target, width, height, margin, innerWidth, innerHeight, xMin, xMax, yMin, yMax, xScale, yScale) {
    const group = el("g");
    const yTicks = state.scale === "log" ? logTicks(yMin, yMax) : linearTicks(yMin, yMax, 5);
    const xTicks = linearTicks(xMin, xMax, 5);

    yTicks.forEach((tick) => {
      const y = yScale(tick);
      group.appendChild(line(margin.left, y, width - margin.right, y, "chart-grid"));
      group.appendChild(text(margin.left - 10, y + 4, formatValue(tick), "chart-label", "end"));
    });

    xTicks.forEach((tick) => {
      const x = xScale(tick);
      group.appendChild(line(x, margin.top, x, height - margin.bottom, "chart-grid"));
      group.appendChild(text(x, height - margin.bottom + 24, formatValue(tick), "chart-label", "middle"));
    });

    group.appendChild(line(margin.left, height - margin.bottom, width - margin.right, height - margin.bottom, "chart-axis"));
    group.appendChild(line(margin.left, margin.top, margin.left, height - margin.bottom, "chart-axis"));
    group.appendChild(text(margin.left + innerWidth / 2, height - 12, state.dataset.xLabel, "chart-label", "middle"));

    const yTitle = text(18, margin.top + innerHeight / 2, `${state.metric.label} (${state.metric.unit})`, "chart-label", "middle");
    yTitle.setAttribute("transform", `rotate(-90 18 ${margin.top + innerHeight / 2})`);
    group.appendChild(yTitle);
    target.appendChild(group);
  }

  function drawSeries(target, series, xScale, yScale) {
    series.forEach((item, index) => {
      const color = colorForSeries(item.label, index);
      const path = item.points
        .map((point, pointIndex) => `${pointIndex === 0 ? "M" : "L"} ${xScale(point.x).toFixed(2)} ${yScale(point.y).toFixed(2)}`)
        .join(" ");
      const pathEl = el("path");
      pathEl.setAttribute("class", "chart-line");
      pathEl.setAttribute("d", path);
      pathEl.setAttribute("stroke", color);
      target.appendChild(pathEl);

      item.points.forEach((point) => {
        const circle = el("circle");
        circle.setAttribute("class", "chart-point");
        circle.setAttribute("cx", xScale(point.x));
        circle.setAttribute("cy", yScale(point.y));
        circle.setAttribute("r", 4.5);
        circle.setAttribute("fill", color);
        circle.addEventListener("mousemove", (event) => showTooltip(event, item.label, point));
        circle.addEventListener("mouseleave", hideTooltip);
        target.appendChild(circle);
      });
    });
  }

  function renderLegend(series) {
    legend.innerHTML = series
      .map((item, index) => {
        const color = colorForSeries(item.label, index);
        return `<span class="legend-item"><span class="legend-swatch" style="background:${color}"></span>${item.label}</span>`;
      })
      .join("");
  }

  function renderTable(series) {
    const rows = series.map((item) => {
      const last = item.points[item.points.length - 1];
      const max = item.points.reduce((best, point) => (point.y > best.y ? point : best), item.points[0]);
      return `<tr><td>${item.label}</td><td>${formatValue(last.x)}</td><td>${formatValue(last.y)} ${state.metric.unit}</td><td>${formatValue(max.y)} ${state.metric.unit}</td></tr>`;
    });
    table.innerHTML = `
      <thead><tr><th>Series</th><th>Final ${state.dataset.xLabel}</th><th>Final Value</th><th>Max Value</th></tr></thead>
      <tbody>${rows.join("")}</tbody>
    `;
  }

  function showTooltip(event, label, point) {
    tooltip.hidden = false;
    tooltip.innerHTML = `<strong>${label}</strong><br>${state.dataset.xLabel}: ${formatValue(point.x)}<br>${state.metric.label}: ${formatValue(point.y)} ${state.metric.unit}`;
    const bounds = event.currentTarget.ownerSVGElement.getBoundingClientRect();
    tooltip.style.left = `${event.clientX - bounds.left + 14}px`;
    tooltip.style.top = `${event.clientY - bounds.top - 10}px`;
  }

  function hideTooltip() {
    tooltip.hidden = true;
  }

  function buildSeries(rows, dataset, metric) {
    if (metric.seriesFromGroups) {
      const groups = new Map();
      rows.forEach((row) => {
        const x = Number(row[dataset.xKey]);
        const y = Number(row[metric.valueKey]);
        const group = row[metric.groupKey];
        if (!Number.isFinite(x) || !Number.isFinite(y)) return;
        if (!groups.has(group)) groups.set(group, []);
        groups.get(group).push({ x, y });
      });
      return Array.from(groups.entries())
        .sort(([a], [b]) => Number(a) - Number(b))
        .map(([group, points]) => ({
          label: `${group} Hz`,
          points: averageByX(points)
        }));
    }

    return metric.series.map(([label, key]) => ({
      label,
      points: averageByX(
        rows
          .map((row) => ({ x: Number(row[dataset.xKey]), y: Number(row[key]) }))
          .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
      )
    }));
  }

  function buildColorMap(series) {
    return new Map(series.map((item, index) => [item.label, colors[index % colors.length]]));
  }

  function colorForSeries(label, fallbackIndex) {
    return state.colorBySeries.get(label) || colors[fallbackIndex % colors.length];
  }

  function averageByX(points) {
    const buckets = new Map();
    points.forEach((point) => {
      if (!buckets.has(point.x)) buckets.set(point.x, []);
      buckets.get(point.x).push(point.y);
    });
    return Array.from(buckets.entries())
      .sort(([a], [b]) => a - b)
      .map(([x, values]) => ({
        x,
        y: values.reduce((sum, value) => sum + value, 0) / values.length
      }));
  }

  function parseHeaderCsv(text) {
    const [headerLine, ...lines] = text.trim().split(/\r?\n/);
    const headers = splitCsvLine(headerLine);
    return lines
      .map(splitCsvLine)
      .filter((cells) => cells.length > 1)
      .map((cells) => Object.fromEntries(headers.map((header, index) => [header, numberOrString(cells[index])])));
  }

  function parseContinuousCsv(text) {
    return text
      .trim()
      .split(/\r?\n/)
      .slice(1)
      .map(splitCsvLine)
      .filter((cells) => cells.length > 12)
      .map((cells) => ({
        agents: Number(cells[0]),
        frequencyHz: Number(cells[1]),
        adgTime: Number(cells[3]),
        sageTime: Number(cells[6]),
        mageTime: Number(cells[7]),
        sageSpeedup: Number(cells[8]),
        mageSpeedup: Number(cells[9]),
        sageComms: Number(cells[11]),
        mageComms: Number(cells[12]),
        mageCommsRatio: Number(cells[13])
      }));
  }

  function splitCsvLine(line) {
    return line.split(",").map((cell) => cell.trim());
  }

  function numberOrString(value) {
    const number = Number(value);
    return Number.isFinite(number) && value !== "" ? number : value;
  }

  function linearTicks(min, max, count) {
    if (min === max) return [min];
    const step = (max - min) / (count - 1);
    return Array.from({ length: count }, (_, index) => min + step * index);
  }

  function logTicks(min, max) {
    const start = Math.floor(Math.log10(min));
    const end = Math.ceil(Math.log10(max));
    const ticks = [];
    for (let power = start; power <= end; power += 1) ticks.push(10 ** power);
    return ticks.filter((tick) => tick >= min && tick <= max);
  }

  function formatValue(value) {
    if (!Number.isFinite(value)) return "";
    if (Math.abs(value) >= 1000) return value.toLocaleString(undefined, { maximumFractionDigits: 0 });
    if (Math.abs(value) >= 10) return value.toLocaleString(undefined, { maximumFractionDigits: 2 });
    if (Math.abs(value) >= 1) return value.toLocaleString(undefined, { maximumFractionDigits: 3 });
    return value.toLocaleString(undefined, { maximumSignificantDigits: 3 });
  }

  function escapeAttribute(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/"/g, "&quot;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;");
  }

  function cssEscape(value) {
    if (window.CSS && CSS.escape) return CSS.escape(value);
    return String(value).replace(/"/g, '\\"');
  }

  function el(name) {
    return document.createElementNS("http://www.w3.org/2000/svg", name);
  }

  function line(x1, y1, x2, y2, className) {
    const element = el("line");
    element.setAttribute("x1", x1);
    element.setAttribute("y1", y1);
    element.setAttribute("x2", x2);
    element.setAttribute("y2", y2);
    element.setAttribute("class", className);
    return element;
  }

  function text(x, y, content, className, anchor) {
    const element = el("text");
    element.setAttribute("x", x);
    element.setAttribute("y", y);
    element.setAttribute("class", className);
    element.setAttribute("text-anchor", anchor);
    element.textContent = content;
    return element;
  }
})();
