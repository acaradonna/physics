import { createEngine } from '../engine.js';

const cfg = { running: true };

function draw(ctx, bodies){
  ctx.clearRect(0,0,ctx.canvas.width,ctx.canvas.height);
  ctx.save();
  ctx.fillStyle = '#a6e22e';
  for (const b of bodies){
    const x = (b.position.x*50)+ctx.canvas.width*0.5;
    const y = ctx.canvas.height*0.6 + (b.position.y*-50);
    const r = (b.radius||0.5)*50;
    ctx.beginPath(); ctx.arc(x,y,r,0,Math.PI*2); ctx.fill();
  }
  ctx.restore();
}

async function main(){
  const { World } = await createEngine();
  const canvas = document.getElementById('collide-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const world = new World();

  const sep = document.getElementById('collide-sep');
  const resetBtn = document.getElementById('collide-reset');
  const toggleBtn = document.getElementById('collide-toggle');

  function reset(){
    world.bodies = [];
    const r = 0.5;
    const a = world.createRigidBody({ position:{x:-0.3,y:0,z:0}, velocity:{x:0,y:0,z:0}, mass:1, radius:r });
    const b = world.createRigidBody({ position:{x:0.3,y:0,z:0}, velocity:{x:0,y:0,z:0}, mass:1, radius:r });
    // annotate for renderer
    world.bodies[a].radius = r; world.bodies[b].radius = r;
  }

  reset();
  resetBtn.addEventListener('click', reset);
  toggleBtn.addEventListener('click', ()=>{ cfg.running = !cfg.running; toggleBtn.textContent = cfg.running ? 'Pause' : 'Resume'; });

  let last = performance.now();
  function loop(t){
    const dt = Math.min(1/60, (t-last)/1000); last = t;
    if (cfg.running){
      world.step(dt);
      // copy separation amount to UI
      const a = world.getPosition(0), b = world.getPosition(1);
      const dx=b.x-a.x, dy=b.y-a.y, dz=b.z-a.z;
      const dist = Math.sqrt(dx*dx+dy*dy+dz*dz);
      if (sep) sep.textContent = dist.toFixed(3);
      // sync positions for renderer
      world.bodies[0].position = a; world.bodies[1].position = b;
    }
    draw(ctx, world.bodies);
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);
}

main();


