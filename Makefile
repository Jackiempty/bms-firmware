GIT_HOOKS := .git/hooks/applied

all: $(GIT_HOOKS)

$(GIT_HOOKS):
	@scripts/install-git-hooks
	@echo