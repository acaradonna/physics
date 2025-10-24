import { createEngine } from '../engine.js';

const cfg = { running: true };

function draw(ctx, bodies) {
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    ctx.save();
    ctx.fillStyle = '#f92672';
    for (const b of bodies) {
        const x = (b.position.x * 50) + ctx.canvas.width * 0.5;
        const y = ctx.canvas.height - (b.position.y * 50);
        const r = (b.radius || 0.5) * 50;
        ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();
}

async function main() {
    const { World } = await createEngine();
    const canvas = document.getElementById('stack-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const world = new World();

    const frictionSlider = document.getElementById('stack-friction');
    const restitutionSlider = document.getElementById('stack-restitution');
    const resetBtn = document.getElementById('stack-reset');
    const toggleBtn = document.getElementById('stack-toggle');

    function reset() {
        world.bodies = [];
        const fric = Number(frictionSlider.value);
        const rest = Number(restitutionSlider.value);
        // Create stack of spheres
        for (let i = 0; i < 5; i++) {
            const id = world.createRigidBody({
                position: { x: 0, y: 0.5 + i * 1.0, z: 0 },
                velocity: { x: 0, y: 0, z: 0 },
                mass: 1, radius: 0.5, friction: fric, restitution: rest
            });
            world.bodies[id] = { position: { x: 0, y: 0.5 + i * 1.0, z: 0 }, radius: 0.5 };
        }
    }

    reset();
    resetBtn.addEventListener('click', reset);
    toggleBtn.addEventListener('click', () => { cfg.running = !cfg.running; toggleBtn.textContent = cfg.running ? 'Pause' : 'Resume'; });

    let last = performance.now();
    function loop(t) {
        const dt = Math.min(1 / 60, (t - last) / 1000); last = t;
        if (cfg.running) {
            world.step(dt);
            // Sync positions from world to renderer
            for (let i = 0; i < world.bodies.length; i++) {
                world.bodies[i].position = world.getPosition(i);
            }
        }
        draw(ctx, world.bodies);
        requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
}

main();

