sdl:
	cargo build --release

wasm:
	cd wasm/www; wasm-pack build && npm start

ci:
	cargo check && cd wasm; cargo check

.PHONY: sdl wasm ci
