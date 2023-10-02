class UserError(Exception):
    def __init__(self, *args, **kwargs) -> None:  # noqa
        super().__init__(*args, **kwargs)
