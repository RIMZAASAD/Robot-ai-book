import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ModernHero from '@site/src/components/ModernHero';
import EnhancedCard from '@site/src/components/EnhancedCard';
import ContentCard from '@site/src/components/ContentCard';

function ModulesSummary() {
  return (
    <section className={clsx('padding-vert--xl', 'container')}>
      <div className="container">
        <h2 className={clsx('text--center', 'margin-bottom--xl', 'text--primary')}>
          Master the Future of Robotics
        </h2>
        <div className="row">
          <div className="col col--6">
            <EnhancedCard
              title="🧠 Embodied Intelligence"
              description="Dive deep into how AI interacts with the physical world, moving beyond digital brains to physical existence."
              variant="glass"
              hoverEffect={true}
            />
          </div>
          <div className="col col--6">
            <EnhancedCard
              title="⚙️ ROS 2 & Simulation"
              description="Build the nervous system of your robots using ROS 2 and validate designs in high-fidelity Gazebo and Isaac Sim environments."
              variant="glass"
              hoverEffect={true}
            />
          </div>
          <div className="col col--6">
            <EnhancedCard
              title="👁️ Vision-Language-Action"
              description="Implement cutting-edge VLA pipelines that allow robots to see, understand commands, and execute complex physical tasks."
              variant="glass"
              hoverEffect={true}
            />
          </div>
          <div className="col col--6">
            <EnhancedCard
              title="🤖 Humanoid Evolution"
              description="From simple bipeds to advanced humanoid systems, explore the mechanical and algorithmic challenges of human-like forms."
              variant="glass"
              hoverEffect={true}
            />
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturedChapters() {
  return (
    <section className={clsx('padding-vert--xl', 'bg--secondary')}>
      <div className="container">
        <h2 className={clsx('text--center', 'margin-bottom--xl', 'text--primary')}>
          Featured Chapters
        </h2>
        <div className="row">
          <div className="col col--4">
            <ContentCard
              title="Introduction to Physical AI"
              description="Learn the fundamental concepts of Physical AI and how it differs from traditional AI approaches."
              to="/docs/chapters/module-1-foundations/chapter-1-introduction-to-physical-ai"
              icon="🧠"
              level="beginner"
              duration="30 min"
              tags={['AI', 'Fundamentals', 'Physical Intelligence']}
            />
          </div>
          <div className="col col--4">
            <ContentCard
              title="ROS 2 Architecture"
              description="Explore the core concepts of ROS 2 and how it serves as the nervous system for robots."
              to="/docs/chapters/module-2-ros/chapter-5-ros2-architecture"
              icon="⚙️"
              level="intermediate"
              duration="45 min"
              tags={['ROS', 'Architecture', 'Systems']}
            />
          </div>
          <div className="col col--4">
            <ContentCard
              title="Vision-Language-Action Systems"
              description="Implement cutting-edge VLA pipelines for advanced robot perception and action."
              to="/docs/chapters/module-4-vla/chapter-17-vla-integration"
              icon="👁️"
              level="advanced"
              duration="60 min"
              tags={['VLA', 'Perception', 'Action']}
            />
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Interactive textbook on Physical AI and Humanoid Robotics with AI-powered learning assistance">
      <ModernHero
        title={siteConfig.title}
        subtitle={siteConfig.tagline}
        description="An interactive textbook covering the fundamentals of Physical AI and Humanoid Robotics, with AI-powered learning assistance to help you master these cutting-edge technologies."
        buttons={[
          {
            text: 'Start Learning 🚀',
            to: '/docs/chapters/module-1-foundations/chapter-1-introduction-to-physical-ai',
            variant: 'secondary'
          },
          {
            text: 'Quick Tutorial 📖',
            to: '/docs/intro',
            variant: 'primary'
          }
        ]}
        showScrollIndicator={true}
      />
      <main>
        <ModulesSummary />
        <FeaturedChapters />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}