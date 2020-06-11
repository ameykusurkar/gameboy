import { init_panic_hook, greet, EmulatorState } from "gameboy";
import { memory } from "gameboy/gameboy_bg";

const COLOR0 = [255, 228, 204];
const COLOR1 = [240, 100, 120];
const COLOR2 = [48, 51, 107];
const COLOR3 = [19, 15, 64];

const COLORS = [COLOR0, COLOR1, COLOR2, COLOR3];

var fileReader = new FileReader();
var fileInputElement = document.getElementById("file-input");
var emu = null;

init_panic_hook();

fileReader.onloadend = function() {
  let buffer = fileReader.result;
  var bytes = new Uint8Array(buffer);

  emu = EmulatorState.new(bytes);
  setInterval(step, 17);
}

fileInputElement.addEventListener("change", e => {
  fileReader.readAsArrayBuffer(fileInputElement.files[0])
});

function step() {
  const t0 = performance.now();
  var canvas = document.getElementById("gameboy-screen");
  var ctx = canvas.getContext("2d");

  emu.clockFrame();

  const pixelsPtr = emu.pixels();
  const pixels = new Uint8Array(memory.buffer, pixelsPtr, 160 * 144);

  var imgData = ctx.createImageData(160, 144);

  for (let i = 0; i < imgData.data.length; i+=4) {
    const pixel = pixels[Math.floor(i / 4)];
    const [r, g, b] = COLORS[pixel];
    imgData.data[i] = r;
    imgData.data[i+1] = g;
    imgData.data[i+2] = b;
    imgData.data[i+3] = 255;
  }

  ctx.putImageData(imgData, 0, 0);
  const t1 = performance.now();
  console.log(t1-t0);
}
