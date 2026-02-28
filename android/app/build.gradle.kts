plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

import java.util.Properties

val localProps = Properties().apply {
    val f = rootProject.file("local.properties")
    if (f.exists()) {
        f.inputStream().use { load(it) }
    }
}

val githubToken: String = (localProps.getProperty("github.token")
    ?: localProps.getProperty("GITHUB_TOKEN")
    ?: ""
).trim()

val githubOwner: String = (localProps.getProperty("github.owner") ?: "atypic").trim()
val githubRepo: String = (localProps.getProperty("github.repo") ?: "wallter").trim()

android {
    namespace = "com.wallter.app"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.wallter.app"
        minSdk = 31
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"

        buildConfigField("String", "GITHUB_OWNER", "\"${githubOwner}\"")
        buildConfigField("String", "GITHUB_REPO", "\"${githubRepo}\"")

        // Default empty; overridden for debug below when a token exists.
        buildConfigField("String", "GITHUB_TOKEN", "\"\"")
    }

    buildFeatures {
        compose = true
        buildConfig = true
    }

    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.14"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
    }

    buildTypes {
        debug {
            if (githubToken.isNotEmpty()) {
                buildConfigField("String", "GITHUB_TOKEN", "\"${githubToken}\"")
            }
        }
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-compose:1.9.3")

    // Provides XML themes such as Theme.Material3.DayNight.NoActionBar
    implementation("com.google.android.material:material:1.12.0")

    implementation(platform("androidx.compose:compose-bom:2024.10.01"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.ui:ui-tooling-preview")
    implementation("androidx.compose.material3:material3")

    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.8.7")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.1")

    debugImplementation("androidx.compose.ui:ui-tooling")
}
