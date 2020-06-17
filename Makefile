sdl:
	cargo build --release

wasm:
	cd wasm/www; wasm-pack build && npm start

.PHONY: sdl wasm
