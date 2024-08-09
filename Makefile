GIT_HOOKS := .git/hooks/applied

all: $(GIT_HOOKS)

$(GIT_HOOKS):
	chmod +x scripts/install-git-hooks
	@scripts/install-git-hooks
	@echo