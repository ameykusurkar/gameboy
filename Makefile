sdl:
	brew install sdl2 sdl2_gfx && cargo build --release

wasm:
	cd wasm/www; wasm-pack build && npm start

ci:
	cargo check && cd wasm; cargo check

test:
	cd core; cargo test

.PHONY: sdl wasm ci test
