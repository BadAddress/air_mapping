import * as THREE from 'three';

export class MapVisualizer {
    constructor(sceneManager) {
        this.scene = sceneManager.scene;
        this.mapChunks = new Map(); // chunk_id -> {points, zMin, zMax}
        this.group = new THREE.Group();
        this.scene.add(this.group);
        this.totalPoints = 0;
        this.isVisible = true;
        this.group.visible = this.isVisible;
        this.colorMode = 'intensity';
        this.colorScratch = new THREE.Color();
        this.zColorMin = -10.0;
        this.zColorMax = 30.0;
    }

    addChunk(chunkId, floatArray) {
        if (this.mapChunks.has(chunkId)) return;

        const pointCount = floatArray.length / 4;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(pointCount * 3);
        const colors = new Float32Array(pointCount * 3);
        const intensities = new Float32Array(pointCount);
        let zMin = Infinity;
        let zMax = -Infinity;
        
        for (let i = 0; i < pointCount; i++) {
            const srcIdx = i * 4;
            const dstIdx = i * 3;
            
            positions[dstIdx] = floatArray[srcIdx];
            positions[dstIdx + 1] = floatArray[srcIdx + 1];
            positions[dstIdx + 2] = floatArray[srcIdx + 2];
            intensities[i] = floatArray[srcIdx + 3];
            zMin = Math.min(zMin, positions[dstIdx + 2]);
            zMax = Math.max(zMax, positions[dstIdx + 2]);
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        geometry.setAttribute('intensity', new THREE.BufferAttribute(intensities, 1));
        geometry.computeBoundingSphere();
        
        const material = new THREE.PointsMaterial({
            size: 1,
            vertexColors: true,
            transparent: true,
            opacity: 1.0,
            sizeAttenuation: false
        });
        
        const points = new THREE.Points(geometry, material);
        this.mapChunks.set(chunkId, { points, zMin, zMax });
        this.group.add(points);
        this.totalPoints += pointCount;
        this.updateColors();

        console.log(`Loaded map chunk ${chunkId}: ${pointCount} points`);
    }

    removeChunk(chunkId) {
        if (this.mapChunks.has(chunkId)) {
            const chunk = this.mapChunks.get(chunkId);
            this.totalPoints -= chunk.points.geometry.attributes.position.count;
            
            this.group.remove(chunk.points);
            chunk.points.geometry.dispose();
            chunk.points.material.dispose();
            this.mapChunks.delete(chunkId);
            this.updateColors();
            
            console.log(`Unloaded map chunk ${chunkId}`);
        }
    }

    clear() {
        for (const chunkId of Array.from(this.mapChunks.keys())) {
            this.removeChunk(chunkId);
        }
        this.totalPoints = 0;
    }

    toggle() {
        this.isVisible = !this.isVisible;
        this.group.visible = this.isVisible;
        return this.isVisible;
    }

    toggleColorMode() {
        this.colorMode = this.colorMode === 'intensity' ? 'z' : 'intensity';
        this.updateColors();
        return this.colorMode;
    }

    updateColors() {
        if (this.mapChunks.size === 0) return;

        const zMin = this.zColorMin;
        const zRange = Math.max(1e-3, this.zColorMax - this.zColorMin);

        for (const chunk of this.mapChunks.values()) {
            const geometry = chunk.points.geometry;
            const positions = geometry.getAttribute('position');
            const colors = geometry.getAttribute('color');
            const intensities = geometry.getAttribute('intensity');

            for (let i = 0; i < positions.count; i++) {
                const z = positions.getZ(i);
                const intensity = intensities.getX(i);
                const color = this.colorMode === 'z'
                    ? this.colorFromZ(z, zMin, zRange, this.colorScratch)
                    : this.colorFromIntensity(intensity, this.colorScratch);

                colors.setXYZ(i, color.r, color.g, color.b);
            }
            colors.needsUpdate = true;
        }
    }

    colorFromIntensity(intensity, target) {
        const t = THREE.MathUtils.clamp(intensity / 255.0, 0.0, 1.0);
        target.setHSL((1.0 - t) * 0.7, 1.0, 0.5);
        return target;
    }

    colorFromZ(z, zMin, zRange, target) {
        const t = THREE.MathUtils.clamp((z - zMin) / zRange, 0.0, 1.0);
        target.setHSL((1.0 - t) * 0.75, 1.0, 0.5);
        return target;
    }
}
