import * as THREE from 'three';
import { CONFIG } from './config.js';
import { PclViewerControls } from './pcl_viewer_controls.js';

export class SceneManager {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.gridHelper = null;
        this.defaultCameraPosition = new THREE.Vector3(1, -1, 0.75).normalize();
        this.defaultTarget = new THREE.Vector3(0, 0, 0);
        this.lastFrameObjects = [];
        this.currentFrameCenter = new THREE.Vector3(0, 0, 0);
        this.currentFrameSize = new THREE.Vector3(100, 100, 10);
        this.currentFrameRadius = 100;
        this.defaultNear = 0.05;
        this.defaultFar = 50000.0;
        this.minCameraDistance = 0.2;
        this.maxCameraDistance = 50000.0;
        
        this.init();
    }

    init() {
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(CONFIG.COLORS.BG);

        // Camera
        const viewport = this.getViewportSize();
        this.camera = new THREE.PerspectiveCamera(
            60,
            viewport.width / viewport.height,
            this.defaultNear,
            this.defaultFar
        );
        this.camera.position.copy(this.defaultCameraPosition).multiplyScalar(250);
        this.camera.up.set(0, 0, 1);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(viewport.width, viewport.height);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);

        // Controls
        this.controls = new PclViewerControls(this.camera, this.renderer.domElement);
        this.controls.rotateSpeed = 0.8;
        this.controls.zoomSpeed = 0.9;
        this.controls.panSpeed = 0.9;
        this.controls.minDistance = this.minCameraDistance;
        this.controls.maxDistance = this.maxCameraDistance;
        this.controls.setTarget(this.defaultTarget);
        this.controls.saveState();

        // Grid
        this.gridHelper = new THREE.GridHelper(1000, 100, CONFIG.COLORS.GRID, CONFIG.COLORS.GRID_CENTER);
        this.gridHelper.rotation.x = Math.PI / 2;
        this.scene.add(this.gridHelper);

        // Lights (Ambient)
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        // Event Listener
        window.addEventListener('resize', () => this.onWindowResize());
        this.renderer.domElement.addEventListener('dblclick', event => this.focusAtPointer(event));
    }

    add(object) {
        this.scene.add(object);
    }

    remove(object) {
        this.scene.remove(object);
    }

    onWindowResize() {
        const viewport = this.getViewportSize();
        this.camera.aspect = viewport.width / viewport.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(viewport.width, viewport.height);
    }

    getViewportSize() {
        return {
            width: Math.max(this.container.clientWidth || window.innerWidth, 1),
            height: Math.max(this.container.clientHeight || window.innerHeight, 1)
        };
    }

    frameObjects(objects) {
        this.lastFrameObjects = Array.from(objects || []);
        const box = new THREE.Box3();
        let hasContent = false;
        for (const object of this.lastFrameObjects) {
            if (!object || object.visible === false) {
                continue;
            }
            const objectBox = new THREE.Box3().setFromObject(object);
            if (objectBox.isEmpty()) {
                continue;
            }
            box.union(objectBox);
            hasContent = true;
        }
        if (!hasContent || box.isEmpty()) {
            this.currentFrameCenter.set(0, 0, 0);
            this.currentFrameSize.set(100, 100, 10);
            this.currentFrameRadius = 100;
            this.resetView();
            return;
        }

        const center = new THREE.Vector3();
        const size = new THREE.Vector3();
        box.getCenter(center);
        box.getSize(size);
        this.currentFrameCenter.copy(center);
        this.currentFrameSize.copy(size);
        this.currentFrameRadius = Math.max(size.length() * 0.5, 1.0);

        this.fitToCurrentFrame();
        this.controls.update();
    }

    fitToCurrentFrame() {
        const viewDirection = this.defaultCameraPosition.clone().normalize();
        this.controls.target.copy(this.currentFrameCenter);
        const fitDistance = this.computeFitDistance();
        this.camera.position
            .copy(this.controls.target)
            .addScaledVector(viewDirection, fitDistance);
        this.camera.lookAt(this.controls.target);
        this.applyCameraLimits();
        this.camera.updateProjectionMatrix();
        this.controls.saveState();
    }

    computeFitDistance() {
        const verticalFov = THREE.MathUtils.degToRad(this.camera.fov);
        const horizontalFov = 2.0 * Math.atan(Math.tan(verticalFov * 0.5) * Math.max(this.camera.aspect, 1e-6));
        const fitHeightDistance = this.currentFrameRadius / Math.sin(verticalFov * 0.5);
        const fitWidthDistance = this.currentFrameRadius / Math.sin(horizontalFov * 0.5);
        return Math.max(fitHeightDistance, fitWidthDistance) * 1.15;
    }

    applyCameraLimits() {
        this.controls.minDistance = this.minCameraDistance;
        this.controls.maxDistance = this.maxCameraDistance;
        this.camera.near = this.defaultNear;
        this.camera.far = this.defaultFar;
    }

    focusAtPointer(event) {
        const focusPoint = this.pickFocusPoint(event);
        if (!focusPoint) {
            return;
        }

        this.controls.focusOn(focusPoint);
    }

    pickFocusPoint(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2.0 - 1.0;
        this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2.0 + 1.0;
        this.raycaster.setFromCamera(this.pointer, this.camera);

        const pickThreshold = THREE.MathUtils.clamp(this.currentFrameRadius * 0.003, 0.5, 8.0);
        this.raycaster.params.Points.threshold = pickThreshold;
        this.raycaster.params.Line.threshold = pickThreshold;

        const candidates = [];
        this.scene.traverse(object => {
            if (!object.visible || object === this.gridHelper) {
                return;
            }
            if (object.isPoints || object.isLine || object.isLineSegments || object.isMesh) {
                candidates.push(object);
            }
        });

        const intersections = this.raycaster.intersectObjects(candidates, false);
        if (intersections.length > 0) {
            return intersections[0].point.clone();
        }

        const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const groundPoint = new THREE.Vector3();
        if (this.raycaster.ray.intersectPlane(groundPlane, groundPoint)) {
            return groundPoint;
        }
        return null;
    }

    render() {
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    resetView() {
        if (this.lastFrameObjects.length > 0) {
            this.frameObjects(this.lastFrameObjects);
            return;
        }

        this.currentFrameCenter.copy(this.defaultTarget);
        this.currentFrameSize.set(100, 100, 10);
        this.currentFrameRadius = 100;
        this.controls.target.copy(this.defaultTarget);
        this.camera.position.copy(this.defaultCameraPosition).multiplyScalar(250);
        this.camera.up.set(0, 0, 1);
        this.controls.setTarget(this.defaultTarget);
        this.applyCameraLimits();
        this.camera.updateProjectionMatrix();
        this.controls.update();
        this.controls.saveState();
    }
}
