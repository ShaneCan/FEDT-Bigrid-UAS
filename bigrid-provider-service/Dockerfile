# ============================= #
# === BUILD STAGE ==============#
# ============================= #
FROM eclipse-temurin:21-jdk-alpine AS build
WORKDIR /build

# Install Maven
RUN apk add --no-cache maven

# Copy project files
COPY . .

# Build the Spring Boot fat JAR (skip tests for speed)
RUN mvn clean package -DskipTests

# ============================= #
# === RUNTIME STAGE ============#
# ============================= #
FROM eclipse-temurin:21-jre-alpine

# Create non-root user
RUN addgroup -S user && adduser -S user -G user
WORKDIR /app

# Copy built JAR (Spring Boot maven plugin puts exec jar in /target)
COPY --from=build /build/bin/bigrid-provider-service.jar /app/bigrid-provider-service.jar

# Set environment variables
ENV JAVA_OPTS="-Xms256m -Xmx512m"
ENV SPRING_PROFILES_ACTIVE=prod

# Switch to non-root user
USER user

# Expose default Spring Boot port
EXPOSE 8080

# Launch the service
ENTRYPOINT ["sh", "-c", "java $JAVA_OPTS -jar bigrid-provider-service.jar"]
