# SERVER22 Codex 0.166.0 platform dependency repair

SSH completed, but the remote Node entrypoint raised `Missing optional dependency
@openai/codex-linux-x64` before the UI. The wrapper package was 0.166.0; its nested
`node_modules/@openai` directory was empty. The earlier check had verified 0.165.0.
No active npm installation was observed; npm omit was empty. The precise operation
that produced this incomplete installation remains unknown.

Installed the matching official alias
`@openai/codex-linux-x64@npm:@openai/codex@0.166.0-linux-x64` from registry.npmjs.org,
with installation scripts disabled, in an isolated directory on the verified
`/data4 /dev/sdc1 ext4` physical data disk. After version validation, created only
its previously absent dependency symlink under the existing wrapper. An unexpected
existing dependency or changed wrapper version would stop the repair. Existing
wrapper files, credentials, account configuration, sessions and running jobs were
preserved. Package/cache files remain on the data disk.

Validation: isolated binary and original global wrapper both returned
`codex-cli 0.166.0`; `/home/yuantao/.local/bin/codex --help` exited successfully.
No full interactive UI, model request, remaining quota, GPU or robotics validation
was performed. The SSH post-quantum advisory was separate from the Node exception.

The remote receipt is
`/data4/yuantao/codex-runtime-repair-20261030-0166/REPAIR_RECEIPT.json`;
parent evidence is external `work/codex22-runtime-repair-20261030/REPAIR_RECEIPT.json`.
Binary size:319346888bytes. Binary SHA256:
`b8a11fedf24b2dfb59a70c9e52b030f9a508dcd49cd2dd0d41956f4c785156b8`.
Npm SRI:
`sha512-CxDm3+D1cpbGJeHrdzINoG+3wn9BVncP4qpZaaNYkxe8iPtRJOYyCJnnPoVjWRIBvI9vPKbRoifsMSA5xDaUpw==`.

Retrievable engineering experience:
`codex22-missing-platform-0166-20261030@1`. The exact original installer failure
remains unresolved. A future global npm reinstall may replace the nested dependency
link; recheck package completeness then. This is a point repair, not a claim of
permanent availability or changes to the robotics result. Rollback only the new
symlink after revalidating its target; do not remove the existing login or sessions.
