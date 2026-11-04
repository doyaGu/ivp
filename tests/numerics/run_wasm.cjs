const fs = require('node:fs');

(async () => {
    const bytes = fs.readFileSync(process.argv[2]);
    const { instance } = await WebAssembly.instantiate(bytes);
    instance.exports.__wasm_call_ctors();
    const failures = instance.exports.ivp_softfloat_arithmetic();
    console.log(`software arithmetic/transcendental/precision failures: ${failures}`);
    process.exitCode = failures ? 1 : 0;
})().catch(error => {
    console.error(error);
    process.exitCode = 1;
});
