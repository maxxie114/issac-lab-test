# Dockerfile for Vultr deployment (no GPU needed)
FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY simulation/ simulation/
COPY api/ api/
COPY frontend/ frontend/
COPY main.py .

EXPOSE 8000

CMD ["python", "main.py", "--port", "8000"]
