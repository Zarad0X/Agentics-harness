"""Fresh initialized continuous diagnostic. Never reconstruct or promote data."""
from physics_runner import main

if __name__=='__main__':
    raise SystemExit(main(continuous=True))
