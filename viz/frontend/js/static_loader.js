export class StaticLoader {
    async getDefaults() {
        return this.getJson('/api/defaults');
    }

    async listFiles(type, root = '') {
        const params = new URLSearchParams({ type, root });
        return this.getJson(`/api/files?${params.toString()}`);
    }

    async loadPcd(path) {
        const params = new URLSearchParams({ path });
        const response = await fetch(`/api/pcd?${params.toString()}`);
        if (!response.ok) {
            throw new Error(`PCD load failed: ${response.status}`);
        }
        const buffer = await response.arrayBuffer();
        const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 4));
        if (magic !== 'PCL1') {
            throw new Error(`Invalid PCD payload magic: ${magic}`);
        }
        return new Float32Array(buffer.slice(4));
    }

    async loadHdMap(path, alignment) {
        const params = new URLSearchParams({ path, alignment });
        return this.getJson(`/api/hdmap?${params.toString()}`);
    }

    async getJson(url) {
        const response = await fetch(url);
        if (!response.ok) {
            throw new Error(`Request failed: ${response.status}`);
        }
        const data = await response.json();
        if (data.ok === false) {
            throw new Error(data.error || 'Request failed');
        }
        return data;
    }
}
