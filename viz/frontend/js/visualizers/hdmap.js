import * as THREE from 'three';

/**
 * HDMapVisualizer - Renders HD map elements with distinct colors
 * 
 * Color scheme:
 * - Crosswalk: White (0xffffff) - closed polygon
 * - Junction: Cyan (0x00ffff) - closed polygon  
 * - Lane center: Yellow (0xffff00) - open line
 * - Lane left boundary: Green (0x00ff00) - open line (solid/dotted based on type)
 * - Lane right boundary: Light green (0x88ff88) - open line
 * - Stop sign: Red (0xff0000) - line/marker
 * - Road boundary: Orange (0xff8800) - open line
 */
export class HDMapVisualizer {
    // HD map Z offset: -1m below point cloud (point cloud Z=0 is at IMU height)
    static HD_MAP_Z_OFFSET = 0.5;
    
    // Color definitions for different HD map element types
    static COLORS = {
        crosswalk: 0xffffff,        // White
        junction: 0x00ffff,         // Cyan
        lane_center: 0xffff00,      // Yellow
        lane_left_boundary: 0x00ff00,   // Green
        lane_right_boundary: 0x88ff88,  // Light green
        stop_sign: 0xff0000,        // Red
        road_boundary: 0xff8800,    // Orange
        // Legacy types for backward compatibility
        lane: 0xffff00,             // Yellow (same as lane_center)
        boundary: 0xff8800,         // Orange
    };
    
    // Boundary type colors (solid vs dotted)
    static BOUNDARY_COLORS = {
        SOLID_WHITE: 0xff00ff,      // Fluorescent pink (no lane change)
        SOLID_YELLOW: 0xff00ff,     // Fluorescent pink (no lane change)
        DOTTED_WHITE: 0xaaaaaa,
        DOTTED_YELLOW: 0xaaaa00,
        DOUBLE_YELLOW: 0xff00ff,    // Fluorescent pink (no lane change)
        CURB: 0x888888,
        UNKNOWN: 0x00ff00,
    };
    
    constructor(sceneManager) {
        this.scene = sceneManager.scene;
        this.group = new THREE.Group();
        this.scene.add(this.group);
        
        this.features = [];  // Store parsed features
        this.isVisible = true;
        this.group.visible = this.isVisible;
        
        console.log('[HDMapVisualizer] Initialized with Z offset:', HDMapVisualizer.HD_MAP_Z_OFFSET);
    }

    /**
     * Process HD map data from server
     * @param {Object} data - Parsed JSON data with features
     */
    loadHDMap(data) {
        console.log('[HDMapVisualizer] ========== LOADING HD MAP ==========');
        console.log('[HDMapVisualizer] Received data:', data);
        
        if (data.type !== 'hdmap') {
            console.warn('[HDMapVisualizer] Invalid data type:', data.type);
            return;
        }
        
        console.log('[HDMapVisualizer] Loading HD map with', data.features?.length || 0, 'features');
        
        // Clear existing features
        this.clear();
        
        // Count by type for logging
        const typeCounts = {};
        
        // Process each feature
        if (data.features && Array.isArray(data.features)) {
            for (const feature of data.features) {
                typeCounts[feature.type] = (typeCounts[feature.type] || 0) + 1;
                this.addFeature(feature);
            }
        }
        
        console.log('[HDMapVisualizer] Feature counts by type:', typeCounts);
        console.log('[HDMapVisualizer] HD map loaded successfully. Total children in group:', this.group.children.length);
        console.log('[HDMapVisualizer] ===============================================');
    }

    /**
     * Add a single HD map feature
     * @param {Object} feature - Feature data with type, id, and points
     */
    addFeature(feature) {
        const { type, id, points, boundary_type } = feature;
        
        if (!points || !Array.isArray(points) || points.length === 0) {
            console.warn('[HDMapVisualizer] Feature has no points:', id);
            return;
        }
        
        switch (type) {
            case 'crosswalk':
                this.addClosedPolygon(id, points, HDMapVisualizer.COLORS.crosswalk, 'crosswalk');
                break;
            case 'junction':
                this.addClosedPolygon(id, points, HDMapVisualizer.COLORS.junction, 'junction');
                break;
            case 'lane_center':
            case 'lane':  // backward compatibility
                this.addOpenLine(id, points, HDMapVisualizer.COLORS.lane_center, 'lane_center');
                break;
            case 'lane_left_boundary':
                this.addLaneBoundary(id, points, boundary_type, 'left');
                break;
            case 'lane_right_boundary':
                this.addLaneBoundary(id, points, boundary_type, 'right');
                break;
            case 'stop_sign':
                this.addStopSign(id, points);
                break;
            case 'road_boundary':
            case 'boundary':  // backward compatibility
                this.addOpenLine(id, points, HDMapVisualizer.COLORS.road_boundary, 'road_boundary');
                break;
            default:
                console.warn('[HDMapVisualizer] Unknown feature type:', type);
                // Fallback: render as open line with default color
                this.addOpenLine(id, points, 0x888888, type);
        }
        
        this.features.push(feature);
    }

    /**
     * Render a closed polygon (crosswalk, junction)
     */
    addClosedPolygon(id, points, color, type) {
        if (points.length < 3) {
            console.warn(`[HDMapVisualizer] ${type} ${id} has insufficient points`);
            return;
        }
        
        const geometry = new THREE.BufferGeometry();
        // Add extra point to close the loop
        const positions = new Float32Array((points.length + 1) * 3);
        
        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            const idx = i * 3;
            
            positions[idx] = point.x;
            positions[idx + 1] = point.y;
            positions[idx + 2] = (point.z || 0) + HDMapVisualizer.HD_MAP_Z_OFFSET;
        }
        
        // Close the loop
        positions[points.length * 3] = points[0].x;
        positions[points.length * 3 + 1] = points[0].y;
        positions[points.length * 3 + 2] = (points[0].z || 0) + HDMapVisualizer.HD_MAP_Z_OFFSET;
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2,
            transparent: true,
            opacity: 0.85
        });
        
        const line = new THREE.LineLoop(geometry, material);
        line.userData = { type: type, id: id };
        this.group.add(line);
    }

    /**
     * Render an open line (lane center, road boundary) - NOT closed
     */
    addOpenLine(id, points, color, type) {
        if (points.length < 2) {
            console.warn(`[HDMapVisualizer] ${type} ${id} has insufficient points`); 
            return;
        }
        
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.length * 3);
        
        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            const idx = i * 3;
            
            positions[idx] = point.x;
            positions[idx + 1] = point.y;
            positions[idx + 2] = (point.z || 0) + HDMapVisualizer.HD_MAP_Z_OFFSET;
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2,
            transparent: true,
            opacity: 0.8
        });
        
        // Use THREE.Line (open line), NOT LineLoop (closed)
        const line = new THREE.Line(geometry, material);
        line.userData = { type: type, id: id };
        this.group.add(line);
    }

    /**
     * Render lane boundary with color based on boundary type
     */
    addLaneBoundary(id, points, boundaryType, side) {
        if (points.length < 2) return;
        
        // Determine color based on boundary type
        let color = side === 'left' 
            ? HDMapVisualizer.COLORS.lane_left_boundary 
            : HDMapVisualizer.COLORS.lane_right_boundary;
        
        if (boundaryType && HDMapVisualizer.BOUNDARY_COLORS[boundaryType]) {
            color = HDMapVisualizer.BOUNDARY_COLORS[boundaryType];
        }
        
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.length * 3);
        
        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            const idx = i * 3;
            
            positions[idx] = point.x;
            positions[idx + 1] = point.y;
            positions[idx + 2] = (point.z || 0) + HDMapVisualizer.HD_MAP_Z_OFFSET;
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const material = new THREE.LineBasicMaterial({
            color: color,
            linewidth: 2,
            transparent: true,
            opacity: 0.8
        });
        
        // Use THREE.Line (open line), NOT LineLoop
        const line = new THREE.Line(geometry, material);
        line.userData = { type: `lane_${side}_boundary`, id: id, boundaryType: boundaryType };
        this.group.add(line);
    }

    /**
     * Render stop sign as a red line
     */
    addStopSign(id, points) {
        if (points.length < 2) return;
        
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(points.length * 3);
        
        for (let i = 0; i < points.length; i++) {
            const point = points[i];
            const idx = i * 3;
            
            positions[idx] = point.x;
            positions[idx + 1] = point.y;
            positions[idx + 2] = (point.z || 0) + HDMapVisualizer.HD_MAP_Z_OFFSET;
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const material = new THREE.LineBasicMaterial({
            color: HDMapVisualizer.COLORS.stop_sign,
            linewidth: 3,
            transparent: true,
            opacity: 0.9
        });
        
        const line = new THREE.Line(geometry, material);
        line.userData = { type: 'stop_sign', id: id };
        this.group.add(line);
    }

    /**
     * Clear all HD map features
     */
    clear() {
        // Remove all children from group
        while (this.group.children.length > 0) {
            const child = this.group.children[0];
            this.group.remove(child);
            
            // Dispose geometry and material to free memory
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }
        
        this.features = [];
        console.log('[HDMapVisualizer] Cleared all features');
    }

    /**
     * Toggle HD map visibility
     * @returns {boolean} New visibility state
     */
    toggle() {
        this.isVisible = !this.isVisible;
        this.group.visible = this.isVisible;
        console.log(`[HDMapVisualizer] Visibility: ${this.isVisible}`);
        return this.isVisible;
    }

    /**
     * Get total number of features
     * @returns {number}
     */
    getFeatureCount() {
        return this.features.length;
    }
}
