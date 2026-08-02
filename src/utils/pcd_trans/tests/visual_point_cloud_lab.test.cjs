const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const htmlPath = path.resolve(__dirname, '..', 'visual_point_cloud_lab.html');
const html = fs.readFileSync(htmlPath, 'utf8');

function loadCore() {
  const match = html.match(/\/\* CLOUDLAB_CORE_START \*\/([\s\S]*?)\/\* CLOUDLAB_CORE_END \*\//);
  assert.ok(match, 'HTML must expose a testable CLOUDLAB_CORE section');
  const context = {
    console,
    TextEncoder,
    TextDecoder,
    Uint8Array,
    Uint16Array,
    Uint32Array,
    Int32Array,
    Float32Array,
    Float64Array,
    ArrayBuffer,
    DataView,
    Math,
    Map,
    Set,
  };
  context.globalThis = context;
  vm.createContext(context);
  vm.runInContext(match[1], context, { filename: 'cloudlab-core.js' });
  return context.CloudLabCore;
}

test('professional workspace exposes resizable dock separators', () => {
  assert.match(html, /data-testid="left-resizer"/);
  assert.match(html, /data-testid="right-resizer"/);
  assert.match(html, /role="separator"/);
});

test('worker source is selected by one explicit core script id', () => {
  assert.match(html, /<script id="cloudlab-core">/);
  assert.match(html, /document\.getElementById\('cloudlab-core'\)/);
  assert.doesNotMatch(html, /find\(script => script\.textContent\.includes\('CLOUDLAB_CORE_START'\)\)/);
  assert.equal((html.match(/CLOUDLAB_CORE_START/g) || []).length, 1);
});

test('viewer gestures restore navigation and support middle-button pan', () => {
  const core = loadCore();
  assert.equal(core.cameraGestureForButton(0, 'none'), 'orbit');
  assert.equal(core.cameraGestureForButton(0, 'box'), 'select');
  assert.equal(core.cameraGestureForButton(1, 'box'), 'pan');
  assert.equal(core.cameraGestureForButton(2, 'none'), 'pan');
  assert.match(html, /setSelectionMode\('none'\)/);
  assert.match(html, /左键旋转 · 中键\/右键平移/);
});

test('voxel filter averages points in the same voxel', () => {
  const core = loadCore();
  const result = core.voxelGridFilter(new Float32Array([
    0.1, 0.1, 0.1,
    0.3, 0.3, 0.3,
    1.2, 0.0, 0.0,
  ]), 1);
  assert.equal(result.length, 6);
  assert.ok(Math.abs(result[0] - 0.2) < 1e-6);
  assert.ok(Math.abs(result[1] - 0.2) < 1e-6);
  assert.ok(Math.abs(result[2] - 0.2) < 1e-6);
  assert.ok(Math.abs(result[3] - 1.2) < 1e-6);
});

test('box crop can keep or remove the selected region', () => {
  const core = loadCore();
  const points = new Float32Array([
    0, 0, 0,
    1, 1, 1,
    3, 3, 3,
  ]);
  const box = { minX: -0.1, maxX: 1.1, minY: -0.1, maxY: 1.1, minZ: -0.1, maxZ: 1.1 };
  assert.deepEqual(Array.from(core.cropByBox(points, box, true)), [0, 0, 0, 1, 1, 1]);
  assert.deepEqual(Array.from(core.cropByBox(points, box, false)), [3, 3, 3]);
});

test('polygon crop uses XY world coordinates and includes boundary points', () => {
  const core = loadCore();
  const points = new Float32Array([
    0, 0, 0,
    1, 1, 0,
    2, 2, 0,
  ]);
  const polygon = [[0, 0], [1, 0], [1, 1], [0, 1]];
  assert.deepEqual(Array.from(core.cropByPolygon(points, polygon, true)), [0, 0, 0, 1, 1, 0]);
});

test('radius outlier filter removes isolated points', () => {
  const core = loadCore();
  const points = new Float32Array([
    0, 0, 0,
    0.1, 0, 0,
    0, 0.1, 0,
    5, 5, 5,
  ]);
  assert.deepEqual(
    Array.from(core.radiusOutlierFilter(points, 0.25, 2)).map(v => Number(v.toFixed(3))),
    [0, 0, 0, 0.1, 0, 0, 0, 0.1, 0],
  );
});

test('geometry classification maps normal angle to the approved semantic classes', () => {
  const core = loadCore();
  const normals = new Float32Array([
    0, 0, 1,
    0.5, 0, Math.sqrt(0.75),
    1, 0, 0,
    Math.sqrt(0.5), 0, Math.sqrt(0.5),
  ]);
  assert.deepEqual(Array.from(core.classifyNormals(normals, {
    groundMaxDeg: 8,
    slopeMaxDeg: 38,
    wallMinDeg: 70,
  })), [core.SEMANTIC.GROUND, core.SEMANTIC.SLOPE, core.SEMANTIC.WALL, core.SEMANTIC.OBSTACLE]);
});

test('Nav2 projection preserves ROS bottom-left origin and PGM top row order', () => {
  const core = loadCore();
  const points = new Float32Array([
    0.2, 0.2, 0.0,
    1.2, 0.2, 0.5,
  ]);
  const labels = new Uint8Array([core.SEMANTIC.GROUND, core.SEMANTIC.OBSTACLE]);
  const grid = core.buildOccupancyGrid(points, labels, {
    minX: 0, minY: 0, maxX: 2, maxY: 2,
    minZ: -1, maxZ: 2, resolution: 1,
    minPointsPerCell: 1,
  });
  assert.equal(grid.width, 2);
  assert.equal(grid.height, 2);
  assert.deepEqual(Array.from(grid.data), [205, 205, 254, 0]);
});

test('signed ESDF uses positive free and negative occupied distances in meters', () => {
  const core = loadCore();
  const esdf = core.buildSignedEsdf(new Uint8Array([254, 0, 254]), 3, 1, {
    resolution: 0.5,
    maxDistance: 5,
    treatUnknownAsObstacle: false,
  });
  assert.deepEqual(Array.from(esdf).map(v => Number(v.toFixed(3))), [0.5, -0.5, 0.5]);
});

test('elevation grid supports minimum and median aggregation', () => {
  const core = loadCore();
  const points = new Float32Array([
    0.2, 0.2, 3,
    0.3, 0.2, 1,
    0.4, 0.2, 2,
  ]);
  const config = { minX: 0, minY: 0, maxX: 1, maxY: 1, resolution: 1 };
  assert.equal(core.buildElevationGrid(points, config, 'min').values[0], 1);
  assert.equal(core.buildElevationGrid(points, config, 'median').values[0], 2);
});

test('map YAML and PGM serializers match Nav2 conventions', () => {
  const core = loadCore();
  const yaml = core.serializeMapYaml('arena.pgm', {
    resolution: 0.05,
    originX: -1.5,
    originY: 2.25,
    occupiedThresh: 0.65,
    freeThresh: 0.25,
  });
  assert.match(yaml, /^image: arena\.pgm/m);
  assert.match(yaml, /^mode: trinary/m);
  assert.match(yaml, /^origin: \[-1\.5, 2\.25, 0\]/m);
  const pgm = core.serializePgm({ width: 2, height: 1, data: new Uint8Array([0, 254]) });
  assert.equal(new TextDecoder().decode(pgm.slice(0, 11)), 'P5\n2 1\n255\n');
  assert.deepEqual(Array.from(pgm.slice(-2)), [0, 254]);
});

test('ESDF binary PCD serializer emits x y z intensity fields', () => {
  const core = loadCore();
  const bytes = core.serializeEsdfPcd(new Float32Array([0.5, -0.5]), {
    width: 2,
    height: 1,
    resolution: 0.1,
    originX: 1,
    originY: 2,
  });
  const text = new TextDecoder().decode(bytes.slice(0, 220));
  assert.match(text, /FIELDS x y z intensity/);
  assert.match(text, /POINTS 2/);
  assert.match(text, /DATA binary/);
});

test('ASCII PCD parser respects field order and ignores non-XYZ fields', () => {
  const core = loadCore();
  const source = [
    'VERSION 0.7',
    'FIELDS intensity z x y',
    'SIZE 4 4 4 4',
    'TYPE F F F F',
    'COUNT 1 1 1 1',
    'WIDTH 2',
    'HEIGHT 1',
    'POINTS 2',
    'DATA ascii',
    '7 3 1 2',
    '8 6 4 5',
    '',
  ].join('\n');
  const parsed = core.parsePcd(new TextEncoder().encode(source).buffer);
  assert.deepEqual(Array.from(parsed.points), [1, 2, 3, 4, 5, 6]);
  assert.deepEqual(Array.from(parsed.intensity), [7, 8]);
});

test('binary PCD parser handles preceding fields and little-endian floats', () => {
  const core = loadCore();
  const header = new TextEncoder().encode([
    'VERSION 0.7',
    'FIELDS normal_x x y z',
    'SIZE 4 4 4 4',
    'TYPE F F F F',
    'COUNT 1 1 1 1',
    'WIDTH 1',
    'HEIGHT 1',
    'POINTS 1',
    'DATA binary',
    '',
  ].join('\n'));
  const body = new Uint8Array(16);
  const view = new DataView(body.buffer);
  [99, 1.5, 2.5, 3.5].forEach((value, index) => view.setFloat32(index * 4, value, true));
  const bytes = new Uint8Array(header.length + body.length);
  bytes.set(header); bytes.set(body, header.length);
  assert.deepEqual(Array.from(core.parsePcd(bytes.buffer).points), [1.5, 2.5, 3.5]);
});

test('compressed PCD parser returns a capability error instead of corrupt points', () => {
  const core = loadCore();
  const source = new TextEncoder().encode('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 1\nDATA binary_compressed\n');
  assert.throws(() => core.parsePcd(source.buffer), /Open3D 后端/);
});

test('surface classifier recognizes a continuous 30 degree ramp', () => {
  const core = loadCore();
  const values = [];
  for (let y = 0; y < 3; y++) {
    for (let x = 0; x < 3; x++) values.push(x, y, x * Math.tan(Math.PI / 6));
  }
  const labels = core.classifySurface(new Float32Array(values), {
    cellSize: 1,
    groundMaxDeg: 8,
    slopeMaxDeg: 38,
    wallMinDeg: 70,
    roughnessMax: 0.2,
  });
  assert.equal(labels[4], core.SEMANTIC.SLOPE);
});

test('oriented box crop rotates the selection around its center', () => {
  const core = loadCore();
  const points = new Float32Array([
    0.6, 0.6, 0,
    0.6, -0.6, 0,
  ]);
  const box = { centerX: 0, centerY: 0, centerZ: 0, sizeX: 2, sizeY: 0.5, sizeZ: 2, yawDeg: 45 };
  assert.deepEqual(
    Array.from(core.cropByOrientedBox(points, box, true)).map(v => Number(v.toFixed(3))),
    [0.6, 0.6, 0],
  );
});

test('occupancy cleanup removes isolated obstacle pixels', () => {
  const core = loadCore();
  const data = new Uint8Array([
    254, 254, 254,
    254,   0, 254,
    254, 254, 254,
  ]);
  const cleaned = core.cleanupOccupancy(data, 3, 3, { removeIsolated: true, fillHoles: false });
  assert.equal(cleaned[4], 254);
});

test('occupancy cleanup fills a one-cell hole enclosed by obstacles', () => {
  const core = loadCore();
  const data = new Uint8Array([
    0, 0, 0,
    0, 254, 0,
    0, 0, 0,
  ]);
  const cleaned = core.cleanupOccupancy(data, 3, 3, { removeIsolated: false, fillHoles: true });
  assert.equal(cleaned[4], 0);
});

test('semantic free-space estimation fills only local unknown gaps away from obstacles', () => {
  const core = loadCore();
  const data = new Uint8Array([254, 205, 205, 205, 0]);
  const semanticFree = new Uint8Array([1, 0, 0, 0, 0]);
  const estimated = core.estimateSemanticFreeSpace(data, 5, 1, semanticFree, {
    radiusCells: 4,
    obstacleClearanceCells: 1,
  });
  assert.deepEqual(Array.from(estimated.data), [254, 254, 254, 205, 0]);
  assert.equal(estimated.estimatedCells, 2);
});

test('unknown-border trim preserves PGM row order and adjusts ROS origin', () => {
  const core = loadCore();
  const trimmed = core.trimUnknownBorder({
    width: 5,
    height: 4,
    data: new Uint8Array([
      205, 205, 205, 205, 205,
      205, 254,   0, 205, 205,
      205, 254, 254, 205, 205,
      205, 205, 205, 205, 205,
    ]),
    resolution: 0.5,
    originX: 10,
    originY: 20,
  }, 0);
  assert.equal(trimmed.width, 2);
  assert.equal(trimmed.height, 2);
  assert.deepEqual(Array.from(trimmed.data), [254, 0, 254, 254]);
  assert.equal(trimmed.originX, 10.5);
  assert.equal(trimmed.originY, 20.5);
});

test('Nav2 projection integrates semantic estimation with automatic border trim', () => {
  const core = loadCore();
  const grid = core.buildOccupancyGrid(
    new Float32Array([1.2, 0.2, 0, 5.2, 0.2, 1]),
    new Uint8Array([core.SEMANTIC.GROUND, core.SEMANTIC.WALL]),
    {
      minX: 0, minY: 0, maxX: 7, maxY: 1,
      minZ: -1, maxZ: 2, resolution: 1,
      estimateSemanticFree: true,
      estimateRadius: 4,
      obstacleClearance: 1,
      autoTrimUnknown: true,
      trimPadding: 0,
    },
  );
  assert.equal(grid.width, 5);
  assert.equal(grid.originX, 1);
  assert.deepEqual(Array.from(grid.data), [254, 254, 254, 205, 0]);
  assert.equal(grid.stats.estimated, 2);
});
