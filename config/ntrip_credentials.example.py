# NTRIP Caster Credentials — template
# Copy this file to ntrip_credentials.py (gitignored) and fill in real
# host/username/password. Never commit the filled-in file.

NTRIP_CASTERS = [
    {
        "name": "Primary (LTE)",
        "host": "",
        "port": 2101,
        "username": "",
        "password": "",
    },
    {
        "name": "Backup (WiFi)",
        "host": "",
        "port": 2101,
        "username": "",
        "password": "",
    },
]

# Recommended for best accuracy:
DEFAULT_MOUNTPOINT = ""
