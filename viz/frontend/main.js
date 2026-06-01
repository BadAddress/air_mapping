import { SceneManager } from './js/scene.js';
import { MapVisualizer } from './js/visualizers/map.js';
import { HDMapVisualizer } from './js/visualizers/hdmap.js';
import { StaticLoader } from './js/static_loader.js';

class AirMappingVizApp {
    constructor() {
        this.sceneManager = new SceneManager('container');
        this.mapViz = new MapVisualizer(this.sceneManager);
        this.hdmapViz = new HDMapVisualizer(this.sceneManager);
        this.loader = new StaticLoader();
        this.storageKey = 'air_mapping_viz_state_v1';
        this.mode = 'local';
        this.frameCount = 0;
        this.lastFrameTime = performance.now();

        this.elements = {
            status: document.getElementById('status'),
            modeLocal: document.getElementById('mode-local'),
            modeGlobal: document.getElementById('mode-global'),
            globalFields: document.getElementById('global-fields'),
            pcdSelect: document.getElementById('pcd-select'),
            pcdPath: document.getElementById('pcd-path'),
            hdmapSelect: document.getElementById('hdmap-select'),
            hdmapPath: document.getElementById('hdmap-path'),
            alignmentSelect: document.getElementById('alignment-select'),
            alignmentPath: document.getElementById('alignment-path'),
            loadBtn: document.getElementById('load-btn'),
            resetBtn: document.getElementById('reset-btn'),
            togglePcdBtn: document.getElementById('toggle-pcd-btn'),
            toggleHdmapBtn: document.getElementById('toggle-hdmap-btn'),
            colorBtn: document.getElementById('color-btn'),
            pcdCount: document.getElementById('pcd-count'),
            hdmapCount: document.getElementById('hdmap-count'),
            utmOffset: document.getElementById('utm-offset'),
            fps: document.getElementById('fps')
        };

        this.bindEvents();
        this.bindPersistenceEvents();
        this.initialize();
        this.animate();
    }

    async initialize() {
        try {
            this.setStatus('Loading file index...');
            const defaults = await this.loader.getDefaults();
            const savedState = this.loadUiState();
            const initialState = {
                mode: savedState?.mode ?? 'local',
                pcdPath: this.resolveDefaultablePath(savedState?.pcdPath, defaults.local_pcd),
                hdmapPath: savedState?.hdmapPath || defaults.hdmap || '',
                alignmentPath: this.resolveDefaultablePath(savedState?.alignmentPath, defaults.utm_alignment),
                colorMode: savedState?.colorMode || 'intensity'
            };
            this.elements.pcdPath.value = initialState.pcdPath;
            this.elements.hdmapPath.value = initialState.hdmapPath;
            this.elements.alignmentPath.value = initialState.alignmentPath;
            await Promise.all([
                this.populateSelect(this.elements.pcdSelect, 'pcd', this.elements.pcdPath),
                this.populateSelect(this.elements.hdmapSelect, 'hdmap', this.elements.hdmapPath),
                this.populateSelect(this.elements.alignmentSelect, 'alignment', this.elements.alignmentPath)
            ]);
            this.setMode(initialState.mode);
            this.applyColorMode(initialState.colorMode);

            const shouldAutoLoad = Boolean(
                savedState &&
                initialState.pcdPath &&
                (initialState.mode !== 'global' || (initialState.hdmapPath && initialState.alignmentPath))
            );
            if (shouldAutoLoad) {
                this.setStatus('Restoring last scene...');
                await this.loadCurrent({ persistState: false });
            } else {
                this.setStatus('Ready');
            }
        } catch (error) {
            this.setStatus(error.message);
            console.error(error);
        }
    }

    bindEvents() {
        this.elements.modeLocal.addEventListener('click', () => this.setMode('local'));
        this.elements.modeGlobal.addEventListener('click', () => this.setMode('global'));
        this.elements.loadBtn.addEventListener('click', () => this.loadCurrent());
        this.elements.resetBtn.addEventListener('click', () => this.sceneManager.resetView());
        this.elements.togglePcdBtn.addEventListener('click', () => {
            const visible = this.mapViz.toggle();
            this.elements.togglePcdBtn.textContent = visible ? '隐藏 PCD' : '显示 PCD';
        });
        this.elements.toggleHdmapBtn.addEventListener('click', () => {
            const visible = this.hdmapViz.toggle();
            this.elements.toggleHdmapBtn.textContent = visible ? '隐藏 HDMap' : '显示 HDMap';
        });
        this.elements.colorBtn.addEventListener('click', () => {
            const mode = this.mapViz.toggleColorMode();
            this.elements.colorBtn.textContent = mode === 'z' ? 'Color: Z-axis' : 'Color: Intensity';
            this.persistUiState();
        });
    }

    bindPersistenceEvents() {
        this.elements.pcdPath.addEventListener('input', () => this.persistUiState());
        this.elements.hdmapPath.addEventListener('input', () => this.persistUiState());
        this.elements.alignmentPath.addEventListener('input', () => this.persistUiState());
        window.addEventListener('beforeunload', () => this.persistUiState());
    }

    fitCurrentScene() {
        this.sceneManager.frameObjects([
            this.mapViz.group,
            this.hdmapViz.group
        ]);
    }

    async populateSelect(select, type, input) {
        const defaults = await this.loader.getDefaults();
        const root = type === 'hdmap'
            ? (defaults.hdmap_root || 'viz/hdmap_local')
            : 'data';
        const data = await this.loader.listFiles(type, root);
        select.innerHTML = '';
        const placeholder = document.createElement('option');
        placeholder.value = '';
        placeholder.textContent = `选择 ${type} 文件...`;
        select.appendChild(placeholder);

        for (const file of data.files || []) {
            const option = document.createElement('option');
            option.value = file.path;
            option.textContent = file.path;
            select.appendChild(option);
        }

        if (input.value) {
            const match = Array.from(select.options).find(option => option.value === input.value);
            if (match) {
                select.value = input.value;
            } else if (type === 'hdmap') {
                input.value = '';
                this.persistUiState();
            }
        }
        select.addEventListener('change', () => {
            if (select.value) {
                input.value = select.value;
                this.persistUiState();
            }
        });
    }

    setMode(mode) {
        this.mode = mode;
        const isGlobal = mode === 'global';
        this.elements.modeLocal.classList.toggle('active', !isGlobal);
        this.elements.modeGlobal.classList.toggle('active', isGlobal);
        this.elements.globalFields.style.display = isGlobal ? 'block' : 'none';
        this.hdmapViz.group.visible = isGlobal && this.hdmapViz.isVisible;
        if (!isGlobal) {
            this.elements.hdmapCount.textContent = '0';
            this.elements.utmOffset.textContent = '-';
        }
        this.persistUiState();
    }

    loadUiState() {
        try {
            const raw = localStorage.getItem(this.storageKey);
            if (!raw) {
                return null;
            }
            const state = JSON.parse(raw);
            if (!state || typeof state !== 'object') {
                return null;
            }

            return {
                mode: state.mode === 'global' ? 'global' : 'local',
                pcdPath: this.normalizeSavedPcdPath(state.pcdPath),
                hdmapPath: this.normalizeSavedHdmapPath(state.hdmapPath),
                alignmentPath: this.normalizeSavedAlignmentPath(state.alignmentPath),
                colorMode: state.colorMode === 'z' ? 'z' : 'intensity'
            };
        } catch (error) {
            console.warn('Failed to load viz UI state:', error);
            return null;
        }
    }

    normalizeSavedHdmapPath(path) {
        if (typeof path !== 'string') {
            return '';
        }
        return path.startsWith('viz/hdmap/') ? '' : path;
    }

    normalizeSavedPcdPath(path) {
        if (typeof path !== 'string') {
            return '';
        }
        if (
            path.endsWith('/stage2_graph_opt/preview/optimized_global_preview.pcd') ||
            path.endsWith('/stage3_graph_refine/preview/refined_global_preview.pcd')
        ) {
            return '';
        }
        const isPreviewPcd = path.includes('/preview/');
        const isStage4GlobalPcd = path.endsWith('/stage4_map_export/global.pcd');
        return path.includes('/keyframes/') || (!isPreviewPcd && !isStage4GlobalPcd)
            ? ''
            : path;
    }

    normalizeSavedAlignmentPath(path) {
        if (typeof path !== 'string') {
            return '';
        }
        return path.includes('/stage2_graph_opt/alignment/utm_origin.txt')
            ? ''
            : path;
    }

    resolveDefaultablePath(savedPath, defaultPath) {
        return savedPath || defaultPath || '';
    }

    persistUiState() {
        try {
            const state = {
                mode: this.mode,
                pcdPath: this.elements.pcdPath.value.trim(),
                hdmapPath: this.elements.hdmapPath.value.trim(),
                alignmentPath: this.elements.alignmentPath.value.trim(),
                colorMode: this.mapViz.colorMode
            };
            localStorage.setItem(this.storageKey, JSON.stringify(state));
        } catch (error) {
            console.warn('Failed to persist viz UI state:', error);
        }
    }

    applyColorMode(mode) {
        const desiredMode = mode === 'z' ? 'z' : 'intensity';
        if (this.mapViz.colorMode !== desiredMode) {
            this.mapViz.toggleColorMode();
        }
        this.elements.colorBtn.textContent = desiredMode === 'z'
            ? 'Color: Z-axis'
            : 'Color: Intensity';
    }

    async loadCurrent(options = {}) {
        const persistState = options.persistState !== false;
        const pcdPath = this.elements.pcdPath.value.trim();
        if (!pcdPath) {
            this.setStatus('PCD path is empty');
            return;
        }

        try {
            this.elements.pcdPath.value = pcdPath;
            this.setStatus('Loading PCD...');
            this.mapViz.clear();
            const points = await this.loader.loadPcd(pcdPath);
            this.mapViz.addChunk(0, points);
            this.elements.pcdCount.textContent = String(points.length / 4);

            if (this.mode === 'global') {
                const hdmapPath = this.elements.hdmapPath.value.trim();
                const alignmentPath = this.elements.alignmentPath.value.trim();
                if (!hdmapPath || !alignmentPath) {
                    throw new Error('GLOBAL mode needs HDMap and utm_alignment');
                }
                this.elements.hdmapPath.value = hdmapPath;
                this.elements.alignmentPath.value = alignmentPath;
                this.setStatus('Loading HDMap...');
                const hdmap = await this.loader.loadHdMap(hdmapPath, alignmentPath);
                this.hdmapViz.loadHDMap(hdmap);
                this.hdmapViz.group.visible = this.hdmapViz.isVisible;
                this.elements.hdmapCount.textContent = String(hdmap.features?.length || 0);
                const offset = hdmap.utm_offset || {};
                this.elements.utmOffset.textContent = [
                    offset.x ?? 0,
                    offset.y ?? 0,
                    offset.z ?? 0
                ].map(v => Number(v).toFixed(3)).join(', ');
            } else {
                this.hdmapViz.clear();
                this.elements.hdmapCount.textContent = '0';
                this.elements.utmOffset.textContent = '-';
            }

            this.fitCurrentScene();
            this.setStatus(`Loaded ${this.mode.toUpperCase()}`);
            if (persistState) {
                this.persistUiState();
            }
        } catch (error) {
            this.setStatus(error.message);
            console.error(error);
        }
    }

    setStatus(text) {
        this.elements.status.textContent = text;
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        this.sceneManager.render();

        this.frameCount++;
        const now = performance.now();
        if (now - this.lastFrameTime >= 1000) {
            this.elements.fps.textContent = String(this.frameCount);
            this.frameCount = 0;
            this.lastFrameTime = now;
        }
    }
}

new AirMappingVizApp();
