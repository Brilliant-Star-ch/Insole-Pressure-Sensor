# Conventional Commits
feat: new feature  
fix: bug fix  
refactor: code change that neither fixes a bug nor adds a feature  
perf: performance improvement  
docs: documentation only changes  
style: formatting/whitespace/semi-colons etc. (no code behavior change)  
test: add/modify tests  
chore: maintenance/tasks (tooling, config, housekeeping)  
build: build system or external dependencies  
ci: CI configuration and scripts  
revert: revert a previous commit  

# php-template
feat(firmware): add master/slave ESP-NOW frame sync  
fix(sd): handle FAT32 init failure and lower SPI clock  
refactor(adc): extract stable-read helper  
perf(logger): batch flush every 500 ms  
docs: add v3 quick start and wiring guide  
style: apply clang-format  
test(merge): add cross-correlation offset tests  
chore: add .gitattributes and normalize line endings  
build: bump esp32 board package to 3.3.0  
ci: add GitHub Actions build matrix  
revert: revert "feat(logger): add side-select build" (abc1234)WW  