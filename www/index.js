import { initPanicHook, greet, EmulatorState } from "gameboy";
import { memory } from "gameboy/gameboy_bg";

const COLOR0 = [255, 228, 204];
const COLOR1 = [240, 100, 120];
const COLOR2 = [48, 51, 107];
const COLOR3 = [19, 15, 64];
const COLORS = [COLOR0, COLOR1, COLOR2, COLOR3];

const MACHINE_CYCLES_PER_SECOND = 1048576;
const TIME_PER_SAMPLE = 1.0 / 44100.0;
const TIME_PER_CLOCK = 1.0 / MACHINE_CYCLES_PER_SECOND;

var fileReader = new FileReader();
var fileInputElement = document.getElementById("file-input");
var emu = null;

var audioCtx = null;

var keysPressed = {
  j: false, // down
  k: false, // up
  h: false, // left
  l: false, // right
  v: false, // select
  n: false, // start
  d: false, // b
  f: false, // a
};

document.addEventListener("keydown", function(e) {
  keysPressed[e.key] = true;
});

document.addEventListener("keyup", function(e) {
  keysPressed[e.key] = false;
});

initCanvas();
initPanicHook();

fileReader.onloadend = function() {
  let buffer = fileReader.result;
  var bytes = new Uint8Array(buffer);

  emu = EmulatorState.new(bytes);

  audioCtx = new AudioContext();
  var scriptProcessor = audioCtx.createScriptProcessor(512, 0, 1);
  scriptProcessor.onaudioprocess = getSamples;
  scriptProcessor.connect(audioCtx.destination);

  requestAnimationFrame(step);
}

fileInputElement.addEventListener("change", e => {
  fileReader.readAsArrayBuffer(fileInputElement.files[0])
});

function step(x) {
  emu.updateJoypad(new Uint8Array([
    keysPressed['j'], // down
    keysPressed['k'], // up
    keysPressed['h'], // left
    keysPressed['l'], // right
    keysPressed['v'], // select
    keysPressed['n'], // start
    keysPressed['d'], // b
    keysPressed['f'], // a
  ]));

  if (emu.isFrameComplete()) {
    const pixelsPtr = emu.pixels();
    const pixels = new Uint8Array(memory.buffer, pixelsPtr, 160 * 144);

    // This is a hack to determine if the pixels are ready, as they
    // should have values in the range 0-3
    if (pixels[0] < 4) {
      updateCanvas(pixels);
    }

    emu.markFrameAsComplete();
  }

  requestAnimationFrame(step);
}

function updateCanvas(pixels) {
  var canvas = document.getElementById("gameboy-screen");
  var ctx = canvas.getContext("2d");

  var imgData = ctx.createImageData(160, 144);

  for (let i = 0; i < imgData.data.length; i+=4) {
    const pixel = pixels[Math.floor(i / 4)];
    imgData.data[i] = COLORS[pixel][0];
    imgData.data[i+1] = COLORS[pixel][1];
    imgData.data[i+2] = COLORS[pixel][2];
    imgData.data[i+3] = 255;
  }

  ctx.putImageData(imgData, 0, 0);
}

function initCanvas() {
  var canvas = document.getElementById("gameboy-screen");
  var ctx = canvas.getContext("2d");
  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, 160, 144);
}

function getSamples(event) {
  var buffer = event.outputBuffer;
  var sampleElapsed = TIME_PER_SAMPLE;
  var clockElapsed = 0.0;

  var data = buffer.getChannelData(0);
  for (let i = 0; i < buffer.length; i++) {
    while (clockElapsed < sampleElapsed) {
      emu.clock();
      clockElapsed += TIME_PER_CLOCK;
    }

    data[i] = emu.currentAudioSample();
    sampleElapsed += TIME_PER_SAMPLE;
  }
}
