import * as THREE from 'three';

export class PclViewerControls {
    constructor(camera, domElement) {
        this.camera = camera;
        this.domElement = domElement;
        this.enabled = true;

        this.target = new THREE.Vector3();
        this.minDistance = 0.2;
        this.maxDistance = 50000.0;
        this.rotateSpeed = 0.8;
        this.panSpeed = 1.0;
        this.zoomSpeed = 1.0;
        this.worldUp = new THREE.Vector3(0, 0, 1);

        this._state = 'none';
        this._lastPointer = new THREE.Vector2();
        this._savedPosition = camera.position.clone();
        this._savedTarget = this.target.clone();
        this._savedUp = camera.up.clone();

        this._handlePointerDown = event => this.onPointerDown(event);
        this._handlePointerMove = event => this.onPointerMove(event);
        this._handlePointerUp = event => this.onPointerUp(event);
        this._handleWheel = event => this.onWheel(event);
        this._handleContextMenu = event => event.preventDefault();

        this.bindEvents();
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
    }

    bindEvents() {
        this.domElement.style.touchAction = 'none';
        this.domElement.addEventListener('pointerdown', this._handlePointerDown);
        this.domElement.addEventListener('pointermove', this._handlePointerMove);
        this.domElement.addEventListener('pointerup', this._handlePointerUp);
        this.domElement.addEventListener('pointercancel', this._handlePointerUp);
        this.domElement.addEventListener('wheel', this._handleWheel, { passive: false });
        this.domElement.addEventListener('contextmenu', this._handleContextMenu);
    }

    dispose() {
        this.domElement.removeEventListener('pointerdown', this._handlePointerDown);
        this.domElement.removeEventListener('pointermove', this._handlePointerMove);
        this.domElement.removeEventListener('pointerup', this._handlePointerUp);
        this.domElement.removeEventListener('pointercancel', this._handlePointerUp);
        this.domElement.removeEventListener('wheel', this._handleWheel);
        this.domElement.removeEventListener('contextmenu', this._handleContextMenu);
    }

    update() {
        return this;
    }

    saveState() {
        this._savedPosition.copy(this.camera.position);
        this._savedTarget.copy(this.target);
        this._savedUp.copy(this.camera.up);
        return this;
    }

    reset() {
        this.camera.position.copy(this._savedPosition);
        this.target.copy(this._savedTarget);
        this.camera.up.copy(this._savedUp);
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
        return this;
    }

    setTarget(target) {
        this.target.copy(target);
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
        return this;
    }

    focusOn(point) {
        const offset = this.camera.position.clone().sub(this.target);
        if (offset.lengthSq() < 1e-9) {
            offset.set(1, -1, 0.75).normalize().multiplyScalar(Math.max(this.minDistance * 10.0, 10.0));
        }

        this.target.copy(point);
        this.camera.position.copy(point).add(offset);
        this.clampDistance();
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
        return this;
    }

    onPointerDown(event) {
        if (!this.enabled) {
            return;
        }

        if (event.button !== 0 && event.button !== 1 && event.button !== 2) {
            return;
        }

        event.preventDefault();
        this._lastPointer.set(event.clientX, event.clientY);

        if (event.button === 0 && event.shiftKey && (event.ctrlKey || event.metaKey)) {
            this._state = 'zoom';
        } else if (event.button === 0 && (event.ctrlKey || event.metaKey)) {
            this._state = 'spin';
        } else if (event.button === 0 && event.shiftKey) {
            this._state = 'pan';
        } else if (event.button === 0) {
            this._state = 'rotate';
        } else if (event.button === 1) {
            this._state = 'pan';
        } else if (event.button === 2) {
            this._state = 'zoom';
        }

        if (this.domElement.setPointerCapture) {
            try {
                this.domElement.setPointerCapture(event.pointerId);
            } catch (error) {
                // Ignore capture errors from synthetic or stale events.
            }
        }
    }

    onPointerMove(event) {
        if (!this.enabled || this._state === 'none') {
            return;
        }

        const deltaX = event.clientX - this._lastPointer.x;
        const deltaY = event.clientY - this._lastPointer.y;
        if (deltaX === 0 && deltaY === 0) {
            return;
        }

        if (this._state === 'rotate') {
            this.rotate(deltaX, deltaY);
        } else if (this._state === 'spin') {
            this.spin(deltaX);
        } else if (this._state === 'pan') {
            this.pan(deltaX, deltaY);
        } else if (this._state === 'zoom') {
            this.dolly(deltaY);
        }

        this._lastPointer.set(event.clientX, event.clientY);
    }

    onPointerUp(event) {
        if (!this.enabled) {
            return;
        }

        if (this.domElement.releasePointerCapture) {
            try {
                this.domElement.releasePointerCapture(event.pointerId);
            } catch (error) {
                // Ignore release errors from synthetic or stale events.
            }
        }

        this._state = 'none';
    }

    onWheel(event) {
        if (!this.enabled) {
            return;
        }

        event.preventDefault();
        const delta = THREE.MathUtils.clamp(event.deltaY, -120.0, 120.0);
        this.dolly(delta);
    }

    rotate(deltaX, deltaY) {
        const viewport = this.getViewportSize();
        const yawAngle = -(2.0 * Math.PI * deltaX * this.rotateSpeed) / Math.max(viewport.width, 1);
        const pitchAngle = -(Math.PI * deltaY * this.rotateSpeed) / Math.max(viewport.height, 1);
        const offset = this.camera.position.clone().sub(this.target);
        const right = new THREE.Vector3().crossVectors(this.camera.getWorldDirection(new THREE.Vector3()), this.worldUp);

        if (right.lengthSq() < 1e-9) {
            right.setFromMatrixColumn(this.camera.matrixWorld, 0);
        }
        right.normalize();

        const yaw = new THREE.Quaternion().setFromAxisAngle(this.worldUp, yawAngle);
        const pitch = new THREE.Quaternion().setFromAxisAngle(right, pitchAngle);
        offset.applyQuaternion(yaw).applyQuaternion(pitch);

        const viewDirection = offset.clone().normalize().negate();
        const upDot = Math.abs(viewDirection.dot(this.worldUp));
        if (upDot > 0.995) {
            return;
        }

        this.camera.position.copy(this.target).add(offset);
        this.camera.up.copy(this.worldUp);
        this.clampDistance();
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
    }

    spin(deltaX) {
        const viewport = this.getViewportSize();
        const rollAngle = -(2.0 * Math.PI * deltaX * this.rotateSpeed) / Math.max(viewport.width, 1);
        const viewDirection = this.camera.getWorldDirection(new THREE.Vector3()).normalize();
        const roll = new THREE.Quaternion().setFromAxisAngle(viewDirection, rollAngle);

        this.camera.up.applyQuaternion(roll).normalize();
        this.orthogonalizeViewUp();
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
    }

    pan(deltaX, deltaY) {
        this.camera.updateMatrixWorld();

        const offset = this.camera.position.clone().sub(this.target);
        const distance = Math.max(offset.length(), this.minDistance);
        const viewport = this.getViewportSize();
        const fov = THREE.MathUtils.degToRad(this.camera.fov);
        const worldPerPixelY = (2.0 * distance * Math.tan(fov * 0.5)) / Math.max(viewport.height, 1);
        const worldPerPixelX = worldPerPixelY * this.camera.aspect;

        const matrix = this.camera.matrixWorld;
        const right = new THREE.Vector3().setFromMatrixColumn(matrix, 0).normalize();
        const up = new THREE.Vector3().setFromMatrixColumn(matrix, 1).normalize();
        const panOffset = new THREE.Vector3();

        panOffset.addScaledVector(right, -deltaX * worldPerPixelX * this.panSpeed);
        panOffset.addScaledVector(up, deltaY * worldPerPixelY * this.panSpeed);

        this.camera.position.add(panOffset);
        this.target.add(panOffset);
        this.clampDistance();
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
    }

    dolly(deltaY) {
        const scale = Math.exp(deltaY * 0.004 * this.zoomSpeed);
        const offset = this.camera.position.clone().sub(this.target);
        const currentDistance = Math.max(offset.length(), 1e-9);
        const nextDistance = THREE.MathUtils.clamp(
            currentDistance * scale,
            this.minDistance,
            this.maxDistance
        );

        offset.setLength(nextDistance);
        this.camera.position.copy(this.target).add(offset);
        this.camera.up.copy(this.worldUp);
        this.camera.lookAt(this.target);
        this.camera.updateMatrixWorld();
    }

    clampDistance() {
        const offset = this.camera.position.clone().sub(this.target);
        const currentDistance = offset.length();
        if (currentDistance < 1e-9) {
            return;
        }

        const nextDistance = THREE.MathUtils.clamp(currentDistance, this.minDistance, this.maxDistance);
        if (Math.abs(nextDistance - currentDistance) < 1e-9) {
            return;
        }

        offset.setLength(nextDistance);
        this.camera.position.copy(this.target).add(offset);
    }

    orthogonalizeViewUp() {
        const viewDirection = this.target.clone().sub(this.camera.position).normalize();
        const right = new THREE.Vector3().crossVectors(viewDirection, this.camera.up);
        if (right.lengthSq() < 1e-9) {
            return;
        }

        right.normalize();
        this.camera.up.crossVectors(right, viewDirection).normalize();
    }

    getViewportSize() {
        return {
            width: Math.max(this.domElement.clientWidth || 1, 1),
            height: Math.max(this.domElement.clientHeight || 1, 1)
        };
    }
}
